/*
 * Nordic_Solar — Stage 4: BLE peripheral (battery + solar status)
 *
 * Combines everything: reads the nPM1300 charger, drives the standard
 * Bluetooth Battery Service (battery %), and publishes a custom service with
 * battery voltage, charge current, charge state, and VBUS/solar presence.
 *
 * Test with "nRF Connect for Mobile":
 *   - Scan for "Nordic_Solar", connect.
 *   - Battery Service shows the level; the custom service (UUID 5c1b0000-...)
 *     exposes the packed status — enable notifications to watch it live.
 *
 * Console (RTT) mirrors the same values for bench debugging.
 */

#include <zephyr/kernel.h>
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

static const struct device *charger = DEVICE_DT_GET(CHARGER_NODE);
static const struct device *status_led = DEVICE_DT_GET(PMIC_LED_NODE);

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

static void status_led_update(const struct solar_status *s)
{
	bool charging = (s->charge_state == SOLAR_CHG_TRICKLE) ||
			(s->charge_state == SOLAR_CHG_CC) ||
			(s->charge_state == SOLAR_CHG_CV);

	if (!device_is_ready(status_led)) {
		return;
	}

	if (charging) {
		led_on(status_led, STATUS_LED_IDX);
	} else {
		led_off(status_led, STATUS_LED_IDX);
	}
}

static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("connection failed (0x%02x)", err);
		return;
	}
	LOG_INF("BLE connected");
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("BLE disconnected (0x%02x)", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = on_connected,
	.disconnected = on_disconnected,
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

	/* BT_LE_ADV_CONN_FAST_1 is the current connectable-advertising preset.
	 * On an older SDK where it is not defined, use BT_LE_ADV_CONN instead.
	 */
	ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
	if (ret) {
		LOG_ERR("advertising start failed (%d)", ret);
		return ret;
	}
	LOG_INF("Advertising as \"%s\"", CONFIG_BT_DEVICE_NAME);

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
