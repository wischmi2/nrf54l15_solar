/*
 * Nordic_Solar — Stage 4: BLE peripheral (battery + solar status)
 *
 * Combines everything: reads the nPM1300 charger, drives the standard
 * Bluetooth Battery Service (battery %), and publishes a custom service with
 * battery voltage, charge current, charge state, and VBUS/solar presence.
 *
 * LED0: solid while trickle/CC/CV; three blinks every 7 s while a phone is
 * connected; one 200 ms heartbeat every 2 s while advertising (after
 * disconnect or at boot); one blink every 5 s when full if not advertising.
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

/* Simple voltage->SoC estimate for a single LiPo cell. Good enough for a
 * status readout; swap in the nRF Fuel Gauge library (CONFIG_NRF_FUEL_GAUGE)
 * for accurate coulomb-counted SoC.
 */
#define VBAT_EMPTY_MV 3300
#define VBAT_FULL_MV  4200

/* LED0 patterns. Charging (solid) wins, then connected, then advertising. */
#define LED_BLINK_ON_MS       200
#define LED_BLINK_GAP_MS      200
#define LED_FULL_BURST        1
#define LED_FULL_PERIOD_MS    5000
#define LED_HB_BURST          1
#define LED_HB_PERIOD_MS      2000
#define LED_CONN_BURST        3
#define LED_CONN_PERIOD_MS    7000

enum status_led_mode {
	LED_MODE_OFF,
	LED_MODE_SOLID,
	LED_MODE_BLINK_FULL,
	LED_MODE_BLINK_HB,
	LED_MODE_BLINK_CONN,
};

static const struct device *charger = DEVICE_DT_GET(CHARGER_NODE);
static const struct device *status_led = DEVICE_DT_GET(PMIC_LED_NODE);
static enum status_led_mode led_mode = LED_MODE_OFF;
static struct solar_status led_status;
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

static enum solar_charge_state decode_state(int status)
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
	if (status & CHG_STATUS_BATTERY_DETECTED) {
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

static void read_pmic(struct solar_status *s)
{
	struct sensor_value v_bat = {0}, i_bat = {0}, chg = {0};

	s->vbus_present = vbus_present() ? 1 : 0;

	if (sensor_sample_fetch(charger) == 0) {
		sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &v_bat);
		sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &i_bat);
		sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_STATUS, &chg);
	}

	float vbat_f = (float)v_bat.val1 + (float)v_bat.val2 / 1000000.0f;
	float ibat_f = (float)i_bat.val1 + (float)i_bat.val2 / 1000000.0f;

	s->vbat_mv = (uint16_t)(vbat_f * 1000.0f);
	s->ibat_ma = (int16_t)(ibat_f * 1000.0f);
	s->charge_state = (uint8_t)decode_state(chg.val1);
	s->battery_pct = soc_from_mv(s->vbat_mv);
}

static bool led_is_blinking(enum status_led_mode mode)
{
	return (mode == LED_MODE_BLINK_FULL) ||
	       (mode == LED_MODE_BLINK_HB) ||
	       (mode == LED_MODE_BLINK_CONN);
}

static bool led_charge_active(uint8_t chg)
{
	return (chg == SOLAR_CHG_TRICKLE) ||
	       (chg == SOLAR_CHG_CC) ||
	       (chg == SOLAR_CHG_CV);
}

static enum status_led_mode led_desired_mode(void)
{
	uint8_t chg = led_status.charge_state;

	if (led_charge_active(chg)) {
		return LED_MODE_SOLID;
	}
	if (atomic_get(&ble_connected)) {
		return LED_MODE_BLINK_CONN;
	}
	if (atomic_get(&ble_advertising)) {
		return LED_MODE_BLINK_HB;
	}
	/* COMPLETE can drop to IDLE while VBUS is still in and the cell is
	 * full. Fallback if advertising is not running.
	 */
	if ((chg == SOLAR_CHG_COMPLETE) ||
	    (led_status.vbus_present && (chg == SOLAR_CHG_IDLE))) {
		return LED_MODE_BLINK_FULL;
	}
	return LED_MODE_OFF;
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
		mode == LED_MODE_SOLID ? "solid-charging" :
		mode == LED_MODE_BLINK_FULL ? "blink-full" :
		mode == LED_MODE_BLINK_HB ? "heartbeat" :
		mode == LED_MODE_BLINK_CONN ? "blink-connected" : "off");

	switch (mode) {
	case LED_MODE_SOLID:
		led_apply(true);
		break;
	case LED_MODE_BLINK_FULL:
		blink_burst = LED_FULL_BURST;
		blink_period_ms = LED_FULL_PERIOD_MS;
		blinks_remaining = blink_burst;
		led_blink_on_fn(NULL);
		break;
	case LED_MODE_BLINK_HB:
		blink_burst = LED_HB_BURST;
		blink_period_ms = LED_HB_PERIOD_MS;
		blinks_remaining = blink_burst;
		led_blink_on_fn(NULL);
		break;
	case LED_MODE_BLINK_CONN:
		blink_burst = LED_CONN_BURST;
		blink_period_ms = LED_CONN_PERIOD_MS;
		blinks_remaining = blink_burst;
		led_blink_on_fn(NULL);
		break;
	default:
		led_apply(false);
		break;
	}
}

static void status_led_update(const struct solar_status *s)
{
	led_status = *s;
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

	LOG_INF("Nordic_Solar bring-up — Stage 4: BLE battery + solar service");

	if (!device_is_ready(charger)) {
		LOG_ERR("nPM1300 charger not ready — check I2C wiring/overlay");
		return -ENODEV;
	}

	ret = bt_enable(NULL);
	if (ret) {
		LOG_ERR("bt_enable failed (%d)", ret);
		return ret;
	}
	LOG_INF("Bluetooth initialized");

	request_advertising();

	while (1) {
		read_pmic(&s);

		bt_bas_set_battery_level(s.battery_pct);
		solar_service_update(&s);
		status_led_update(&s);

		LOG_INF("VBAT %u mV (%u%%) | I %d mA | %s | VBUS %s",
			s.vbat_mv, s.battery_pct, s.ibat_ma,
			state_str((enum solar_charge_state)s.charge_state),
			s.vbus_present ? "present" : "absent");

		k_msleep(2000);
	}

	return 0;
}
