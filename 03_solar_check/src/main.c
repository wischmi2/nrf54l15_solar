/*
 * Nordic_Solar — Stage 3: solar / VBUS input check + battery read
 *
 * "Is the solar working?" — what the nPM1300 can actually tell you:
 *
 * On this board, USB-C (via D1) and the SPV1040T solar output (via D2) are
 * diode-OR'd into a single CHG_IN/VBUS node. The PMIC sees ONE input; it
 * cannot electrically tell solar from USB. So firmware reports:
 *   - VBUS present?        (is any input powering CHG_IN)
 *   - charging + current   (is that input actually delivering charge)
 *   - battery voltage       (is the cell responding)
 *
 * To confirm SOLAR specifically, use this test:
 *   1. Unplug USB-C. Cover the panel  -> VBUS should read NOT present.
 *   2. Uncover the panel in good light -> VBUS present + charge current > 0.
 * That transition is proof the SPV1040T -> D2 -> CHG_IN path is live.
 *
 * (The nPM1300 exposes VBUS voltage on its ADC, but the mainline Zephyr driver
 *  surfaces VBUS as present/absent + status, not as a calibrated voltage, so we
 *  report presence rather than a made-up solar-voltage number.)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(solar_check, LOG_LEVEL_INF);

#define CHARGER_NODE DT_NODELABEL(npm1300_charger)

#define CHG_STATUS_BATTERY_DETECTED BIT(0)
#define CHG_STATUS_COMPLETE         BIT(1)
#define CHG_STATUS_TRICKLE          BIT(2)
#define CHG_STATUS_CC               BIT(3)
#define CHG_STATUS_CV               BIT(4)

#define VBUS_PRESENT     BIT(0)
#define VBUS_CUR_LIMIT   BIT(1)
#define VBUS_OVP         BIT(2)
#define VBUS_UV          BIT(3)

static float sv_to_float(const struct sensor_value *v)
{
	return (float)v->val1 + (float)v->val2 / 1000000.0f;
}

int main(void)
{
	const struct device *charger = DEVICE_DT_GET(CHARGER_NODE);
	struct sensor_value v_bat, i_bat, chg_status, chg_err, vbus_stat;
	bool last_present = false;
	bool first = true;

	LOG_INF("Nordic_Solar bring-up — Stage 3: solar / VBUS check");

	if (!device_is_ready(charger)) {
		LOG_ERR("nPM1300 charger not ready — check I2C wiring/overlay");
		return -ENODEV;
	}

	LOG_INF("Watching VBUS voltage-present (not USB CC). Unplug USB; sun on panel.");
	LOG_INF("nPM1300 needs ~4.0-5.5 V on CHG_IN; 4.0 V is the floor and often will not charge.");

	while (1) {
		if (sensor_sample_fetch(charger) < 0) {
			LOG_ERR("sample fetch failed");
			k_msleep(2000);
			continue;
		}

		sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &v_bat);
		sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &i_bat);
		sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_STATUS, &chg_status);
		sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_ERROR, &chg_err);
		sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS, &vbus_stat);

		float i_ma = sv_to_float(&i_bat) * 1000.0f;
		bool present = (vbus_stat.val1 & VBUS_PRESENT) != 0;
		bool charging = (chg_status.val1 &
				 (CHG_STATUS_TRICKLE | CHG_STATUS_CC | CHG_STATUS_CV)) != 0;

		if (first || present != last_present) {
			LOG_INF("--- VBUS input %s ---",
				present ? "PRESENT (USB and/or solar supplying)"
					: "ABSENT (running from battery)");
			first = false;
			last_present = present;
		}

		LOG_INF("VBUS:%s UV:%d OVP:%d  charging:%s  I:%.1f mA  VBAT:%.3f V  "
			"chg=0x%02x err=0x%02x vbus=0x%02x",
			present ? "yes" : "no ",
			(vbus_stat.val1 & VBUS_UV) != 0,
			(vbus_stat.val1 & VBUS_OVP) != 0,
			charging ? "yes" : "no ",
			(double)i_ma,
			(double)sv_to_float(&v_bat),
			chg_status.val1, chg_err.val1, vbus_stat.val1);

		if (present && charging && i_ma > 1.0f) {
			LOG_INF("  -> input is delivering charge. If USB is unplugged, "
				"this is your solar panel working.");
		} else if (present && !charging) {
			LOG_INF("  -> VBUS seen but not charging (need >4 V under load; "
				"USB was 5 V). Check SPV1040T VOUT setpoint.");
		}

		k_msleep(2000);
	}

	return 0;
}
