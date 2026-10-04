/*
 * Nordic_Solar — Stage 4: BLE peripheral (battery + solar status)
 *
 * Combines everything: reads the nPM1300 charger, drives the standard
 * Bluetooth Battery Service (battery %), and publishes a custom service with
 * battery voltage, charge current, charge state, and VBUS/solar presence.
 *
 * LED0: solid as a bench load; 1 Hz blink while AEM is charging the cell.
 *
 * Test with "nRF Connect for Mobile":
 *   - Scan for "Nordic_Solar", connect.
 *   - Battery Service shows the level; the custom service (UUID 5c1b0000-...)
 *     exposes the packed status — enable notifications to watch it live.
 *
 * Console (RTT) mirrors the same values for bench debugging.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/logging/log.h>

#include "solar_service.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define CHARGER_NODE    DT_NODELABEL(npm1300_charger)
#define PMIC_LED_NODE   DT_NODELABEL(npm1300_leds)
#define STATUS_LED_IDX  0

#define CHG_STATUS_BATTERY_DETECTED BIT(0)
#define CHG_STATUS_COMPLETE         BIT(1)
#define CHG_STATUS_TRICKLE          BIT(2)
#define CHG_STATUS_CC               BIT(3)
#define CHG_STATUS_CV               BIT(4)

#define VBUS_PRESENT   BIT(0)

/* AEM10330 STO_CFG=LLLL (v3): charge to 4.05 V, ready 3.50 V, cut-off 3.00 V.
 * USB via nPM1300 still terminates at 4.20 V, so a USB-full cell is AEM-BLOCKED.
 */
#define AEM_VOVCH_MV        4050
#define AEM_VCHRDY_MV       3500
#define AEM_VOVDIS_MV       3000
#define AEM_IBAT_CHG_MA     3
#define VBAT_PRESENT_MIN_MV 2500
#define VBAT_PRESENT_MAX_MV 4500

/* Simple voltage->SoC estimate for a single LiPo cell. Good enough for a
 * status readout; swap in the nRF Fuel Gauge library (CONFIG_NRF_FUEL_GAUGE)
 * for accurate coulomb-counted SoC.
 */
#define VBAT_EMPTY_MV 3300
#define VBAT_FULL_MV  4200

/* LED0: solid until AEM charge, then one 200 ms blink each second. */
#define LED_BLINK_ON_MS       200
#define LED_BLINK_GAP_MS      200
#define LED_AEM_BURST         1
#define LED_AEM_PERIOD_MS     1000

enum status_led_mode {
	LED_MODE_OFF,
	LED_MODE_SOLID,
	LED_MODE_BLINK_AEM,
};

static const struct device *charger = DEVICE_DT_GET(CHARGER_NODE);
static const struct device *status_led = DEVICE_DT_GET(PMIC_LED_NODE);
static enum status_led_mode led_mode = LED_MODE_OFF;
static bool led_aem_charging;
static atomic_t ble_connected;
static atomic_t ble_advertising;
static uint8_t blink_burst;
static uint8_t blinks_remaining;
static uint32_t blink_period_ms;
static void led_blink_on_fn(struct k_work *work);
static void led_blink_off_fn(struct k_work *work);
static void led_refresh_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(led_blink_on_work, led_blink_on_fn);
static K_WORK_DELAYABLE_DEFINE(led_blink_off_work, led_blink_off_fn);
static K_WORK_DEFINE(led_refresh_work, led_refresh_fn);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static uint8_t soc_from_mv(uint16_t mv)
{
	if (mv <= VBAT_EMPTY_MV) {
		return 0;
	}
	if (mv >= VBAT_FULL_MV) {
		return 100;
	}
	return (uint8_t)(((uint32_t)(mv - VBAT_EMPTY_MV) * 100) /
			 (VBAT_FULL_MV - VBAT_EMPTY_MV));
}

static enum solar_charge_state decode_state(int status, uint16_t vbat_mv)
{
	if (status & CHG_STATUS_COMPLETE) {
		return SOLAR_CHG_COMPLETE;
	}
	if (status & CHG_STATUS_CV) {
		return SOLAR_CHG_CV;
	}
	if (status & CHG_STATUS_CC) {
		return SOLAR_CHG_CC;
	}
	if (status & CHG_STATUS_TRICKLE) {
		return SOLAR_CHG_TRICKLE;
	}
	/* BIT(0) BATTERYDETECTED is reserved/always 0 on this nPM1300 path.
	 * A real LiPo voltage means the cell is present even when idle.
	 */
	if ((status & CHG_STATUS_BATTERY_DETECTED) ||
	    ((vbat_mv >= VBAT_PRESENT_MIN_MV) && (vbat_mv <= VBAT_PRESENT_MAX_MV))) {
		return SOLAR_CHG_IDLE;
	}
	return SOLAR_CHG_NO_BATTERY;
}

static const char *state_str(enum solar_charge_state s)
{
	switch (s) {
	case SOLAR_CHG_COMPLETE: return "COMPLETE";
	case SOLAR_CHG_CV:       return "CV";
	case SOLAR_CHG_CC:       return "CC";
	case SOLAR_CHG_TRICKLE:  return "TRICKLE";
	case SOLAR_CHG_IDLE:     return "IDLE";
	default:                 return "NO-BATT";
	}
}

static bool vbus_present(void)
{
	struct sensor_value val = {0};

	if (sensor_attr_get(charger,
			    (enum sensor_channel)SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS,
			    (enum sensor_attribute)SENSOR_ATTR_NPM13XX_CHARGER_VBUS_PRESENT,
			    &val) < 0) {
		return false;
	}
	return val.val1 != 0;
}

/* USB-C CC detect (not VBUS). Solar through the SPV1040T has no CC. */
static bool usb_cc_present(void)
{
	struct sensor_value val = {0};

	if (sensor_attr_get(charger, SENSOR_CHAN_CURRENT,
			    SENSOR_ATTR_UPPER_THRESH, &val) < 0) {
		return false;
	}
	return (val.val1 != 0) || (val.val2 != 0);
}

static bool charge_active(uint8_t chg)
{
	return (chg == SOLAR_CHG_TRICKLE) ||
	       (chg == SOLAR_CHG_CC) ||
	       (chg == SOLAR_CHG_CV);
}

static const char *aem_window_str(uint16_t mv)
{
	if (mv > AEM_VOVCH_MV) {
		return "AEM-BLOCKED";
	}
	if (mv >= AEM_VCHRDY_MV) {
		return "AEM-OK";
	}
	if (mv >= AEM_VOVDIS_MV) {
		return "AEM-LOW";
	}
	return "AEM-CUTOFF";
}

static const char *input_src_str(bool vbus, bool usb_cc, bool charging, bool aem_chg)
{
	if (vbus) {
		if (usb_cc) {
			return charging ? "USB" : "USB-IDLE";
		}
		return charging ? "USB-NO-CC" : "USB-NO-CC-IDLE";
	}
	return aem_chg ? "AEM" : "BATTERY";
}

static int read_pmic(struct solar_status *s, bool *usb_cc, bool *aem_chg,
		     int *chg_raw, int *err_raw, int *vbus_raw)
{
	struct sensor_value v_bat = {0}, i_bat = {0}, chg = {0}, err = {0}, vbus = {0};
	int ret;

	s->vbus_present = vbus_present() ? 1 : 0;
	*usb_cc = usb_cc_present();

	ret = sensor_sample_fetch(charger);
	if (ret < 0) {
		s->vbat_mv = 0;
		s->ibat_ma = 0;
		s->charge_state = SOLAR_CHG_NO_BATTERY;
		s->battery_pct = 0;
		*aem_chg = false;
		*chg_raw = 0;
		*err_raw = 0;
		*vbus_raw = 0;
		return ret;
	}

	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &v_bat);
	sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &i_bat);
	sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_STATUS, &chg);
	sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_ERROR, &err);
	sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS, &vbus);

	float vbat_f = (float)v_bat.val1 + (float)v_bat.val2 / 1000000.0f;
	float ibat_f = (float)i_bat.val1 + (float)i_bat.val2 / 1000000.0f;

	s->vbat_mv = (uint16_t)(vbat_f * 1000.0f);
	s->ibat_ma = (int16_t)(ibat_f * 1000.0f);
	s->charge_state = (uint8_t)decode_state(chg.val1, s->vbat_mv);
	s->battery_pct = soc_from_mv(s->vbat_mv);
	s->vbus_present = (vbus.val1 & VBUS_PRESENT) ? 1 : s->vbus_present;
	*aem_chg = !s->vbus_present && (s->ibat_ma >= AEM_IBAT_CHG_MA);
	*chg_raw = chg.val1;
	*err_raw = err.val1;
	*vbus_raw = vbus.val1;
	return 0;
}

static bool led_is_blinking(enum status_led_mode mode)
{
	return mode == LED_MODE_BLINK_AEM;
}

static enum status_led_mode led_desired_mode(void)
{
	if (led_aem_charging) {
		return LED_MODE_BLINK_AEM;
	}
	return LED_MODE_SOLID;
}

static void led_apply(bool on)
{
	if (!device_is_ready(status_led)) {
		return;
	}

	if (on) {
		led_on(status_led, STATUS_LED_IDX);
	} else {
		led_off(status_led, STATUS_LED_IDX);
	}
}

static void led_blink_stop(void)
{
	(void)k_work_cancel_delayable(&led_blink_on_work);
	(void)k_work_cancel_delayable(&led_blink_off_work);
}

static void led_blink_on_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!led_is_blinking(led_mode)) {
		return;
	}

	led_apply(true);
	k_work_schedule(&led_blink_off_work, K_MSEC(LED_BLINK_ON_MS));
}

static void led_blink_off_fn(struct k_work *work)
{
	uint32_t used_ms;
	uint32_t rest_ms;

	ARG_UNUSED(work);

	if (!led_is_blinking(led_mode)) {
		return;
	}

	led_apply(false);

	if (blinks_remaining > 0) {
		blinks_remaining--;
	}

	if (blinks_remaining > 0) {
		k_work_schedule(&led_blink_on_work, K_MSEC(LED_BLINK_GAP_MS));
		return;
	}

	blinks_remaining = blink_burst;
	used_ms = (uint32_t)blink_burst * LED_BLINK_ON_MS;
	if (blink_burst > 0) {
		used_ms += (uint32_t)(blink_burst - 1) * LED_BLINK_GAP_MS;
	}
	rest_ms = (blink_period_ms > used_ms) ? (blink_period_ms - used_ms) : 0;
	k_work_schedule(&led_blink_on_work, K_MSEC(rest_ms));
}

static void led_refresh_fn(struct k_work *work)
{
	enum status_led_mode mode = led_desired_mode();

	ARG_UNUSED(work);

	if (mode == led_mode) {
		return;
	}

	led_mode = mode;
	led_blink_stop();
	LOG_INF("LED mode %s",
		mode == LED_MODE_SOLID ? "solid" :
		mode == LED_MODE_BLINK_AEM ? "blink-aem" : "off");

	switch (mode) {
	case LED_MODE_SOLID:
		led_apply(true);
		break;
	case LED_MODE_BLINK_AEM:
		blink_burst = LED_AEM_BURST;
		blink_period_ms = LED_AEM_PERIOD_MS;
		blinks_remaining = blink_burst;
		led_blink_on_fn(NULL);
		break;
	default:
		led_apply(false);
		break;
	}
}

static void status_led_update(const struct solar_status *s, bool aem_chg)
{
	ARG_UNUSED(s);
	led_aem_charging = aem_chg;
	(void)k_work_submit(&led_refresh_work);
}

static void status_led_set_connected(bool connected)
{
	atomic_set(&ble_connected, connected ? 1 : 0);
	(void)k_work_submit(&led_refresh_work);
}

static int start_advertising(void)
{
	int ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
				  NULL, 0);

	if (ret && ret != -EALREADY) {
		atomic_set(&ble_advertising, 0);
		LOG_ERR("advertising start failed (%d)", ret);
		(void)k_work_submit(&led_refresh_work);
		return ret;
	}
	atomic_set(&ble_advertising, 1);
	(void)k_work_submit(&led_refresh_work);
	LOG_INF("Advertising as \"%s\"", CONFIG_BT_DEVICE_NAME);
	return 0;
}

/* NCS 3.x: do not call bt_le_adv_start() from disconnected — the conn
 * object is still held (CONFIG_BT_MAX_CONN=1 -> -ENOMEM). Wait for
 * .recycled, then start from a work item (callback is ISR-like).
 */
static void adv_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	(void)start_advertising();
}

static K_WORK_DEFINE(adv_work, adv_work_handler);

static void request_advertising(void)
{
	(void)k_work_submit(&adv_work);
}

static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("connection failed (0x%02x)", err);
		request_advertising();
		return;
	}
	LOG_INF("BLE connected");
	atomic_set(&ble_advertising, 0);
	status_led_set_connected(true);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	LOG_INF("BLE disconnected (0x%02x)", reason);
	/* Heartbeat immediately; recycled will restart advertising. */
	atomic_set(&ble_advertising, 1);
	status_led_set_connected(false);
}

static void on_recycled(void)
{
	request_advertising();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = on_connected,
	.disconnected = on_disconnected,
	.recycled = on_recycled,
};

int main(void)
{
	struct solar_status s;
	int ret;
	int last_vbat_mv = -1;

	LOG_INF("Nordic_Solar bring-up — Stage 4: BLE battery + AEM/solar service");

	if (!device_is_ready(charger)) {
		LOG_ERR("nPM1300 charger not ready — check I2C wiring/overlay");
		return -ENODEV;
	}

	led_apply(true);
	led_mode = LED_MODE_SOLID;
	LOG_INF("LED held on");

	ret = bt_enable(NULL);
	if (ret) {
		LOG_ERR("bt_enable failed (%d)", ret);
		return ret;
	}
	LOG_INF("Bluetooth initialized");

	request_advertising();

	while (1) {
		bool usb_cc = false;
		bool aem_chg = false;
		int chg_raw = 0;
		int err_raw = 0;
		int vbus_raw = 0;
		int dv_mv = 0;

		ret = read_pmic(&s, &usb_cc, &aem_chg, &chg_raw, &err_raw, &vbus_raw);
		if (ret < 0) {
			LOG_ERR("nPM1300 sample fetch failed (%d)", ret);
			k_msleep(2000);
			continue;
		}

		bt_bas_set_battery_level(s.battery_pct);
		solar_service_update(&s);
		status_led_update(&s, aem_chg);

		if (last_vbat_mv >= 0) {
			dv_mv = (int)s.vbat_mv - last_vbat_mv;
		}
		last_vbat_mv = s.vbat_mv;

		LOG_INF("SRC:%s | VBUS %s | USB-CC %s | %s | I %+d mA %s | "
			"VBAT %u mV dV %+d | %s | chg=0x%02x err=0x%02x vbus=0x%02x",
			input_src_str(s.vbus_present, usb_cc,
				      charge_active(s.charge_state), aem_chg),
			s.vbus_present ? "yes" : "no",
			usb_cc ? "yes" : "no",
			state_str((enum solar_charge_state)s.charge_state),
			s.ibat_ma,
			(s.ibat_ma > 0) ? "chg" : ((s.ibat_ma < 0) ? "dis" : "idle"),
			s.vbat_mv, dv_mv, aem_window_str(s.vbat_mv),
			chg_raw, err_raw, vbus_raw);

		k_msleep(2000);
	}

	return 0;
}
