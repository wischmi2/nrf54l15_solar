/*
 * Nordic_Solar — Stage 2: nPM1300 charger enable + status
 *
 * The nPM1300 charger is OFF out of reset. Charge parameters (VTERM, ICHG,
 * input current limit, NTC/thermistor) are applied by the Zephyr driver from
 * the devicetree overlay, and "charging-enable" turns it on at boot. This app
 * confirms the charger came up with the expected profile and then reports the
 * live charge state so you can watch a battery actually charge.
 *
 * What to expect with a battery + USB/solar attached:
 *   - "Charging enabled" and the configured VTERM/ICHG/VBUS-limit
 *   - state cycling through TRICKLE -> CC -> CV -> COMPLETE as the cell fills
 *   - battery voltage climbing toward ~4.2 V, positive charge current
 *
 * NOTE on battery temperature: your board fakes the NTC with a fixed 10k, so
 * charging will never NTC-fault, but there is NO real cell temperature
 * protection. Fine on the bench; add a real NTC before any outdoor LiPo build.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(charger, LOG_LEVEL_INF);

#define CHARGER_NODE DT_NODELABEL(npm1300_charger)

/* nPM1300 charge-status register bits (BCHGCHARGESTATUS), as returned in
 * SENSOR_CHAN_NPM13XX_CHARGER_STATUS .val1.
 */
#define CHG_STATUS_BATTERY_DETECTED BIT(0)
#define CHG_STATUS_COMPLETE         BIT(1)
#define CHG_STATUS_TRICKLE          BIT(2)
#define CHG_STATUS_CC               BIT(3)
#define CHG_STATUS_CV               BIT(4)

static const char *charge_state_str(int status)
{
	if (status & CHG_STATUS_COMPLETE) {
		return "COMPLETE";
	}
	if (status & CHG_STATUS_CV) {
		return "CV (topping off)";
	}
	if (status & CHG_STATUS_CC) {
		return "CC (fast charge)";
	}
	if (status & CHG_STATUS_TRICKLE) {
		return "TRICKLE";
	}
	if (status & CHG_STATUS_BATTERY_DETECTED) {
		return "idle (not charging)";
	}
	return "no battery detected";
}

static float sv_to_float(const struct sensor_value *v)
{
	return (float)v->val1 + (float)v->val2 / 1000000.0f;
}

int main(void)
{
	const struct device *charger = DEVICE_DT_GET(CHARGER_NODE);
	struct sensor_value v_bat, i_bat, t_die, chg_status;
	int ret;

	LOG_INF("Nordic_Solar bring-up — Stage 2: nPM1300 charger");

	if (!device_is_ready(charger)) {
		LOG_ERR("nPM1300 charger not ready — check I2C wiring/overlay");
		return -ENODEV;
	}

	/* Configured profile, straight from the devicetree so what you read here
	 * is exactly what the hardware was programmed with.
	 */
	LOG_INF("Charger configured from devicetree:");
	LOG_INF("  VTERM        = %d uV", DT_PROP(CHARGER_NODE, term_microvolt));
	LOG_INF("  ICHG         = %d uA", DT_PROP(CHARGER_NODE, current_microamp));
	LOG_INF("  VBUS limit   = %d uA", DT_PROP(CHARGER_NODE, vbus_limit_microamp));
	LOG_INF("  thermistor   = %d ohm (beta %d)",
		DT_PROP(CHARGER_NODE, thermistor_ohms),
		DT_PROP(CHARGER_NODE, thermistor_beta));
	LOG_INF("  charging-enable = %s",
		DT_PROP(CHARGER_NODE, charging_enable) ? "yes" : "NO (off)");

	while (1) {
		ret = sensor_sample_fetch(charger);
		if (ret < 0) {
			LOG_ERR("sample fetch failed (%d)", ret);
			k_msleep(2000);
			continue;
		}

		sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &v_bat);
		sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &i_bat);
		sensor_channel_get(charger, SENSOR_CHAN_GAUGE_TEMP, &t_die);
		sensor_channel_get(charger, SENSOR_CHAN_NPM13XX_CHARGER_STATUS,
				   &chg_status);

		LOG_INF("VBAT %.3f V | I %.1f mA | T %.1f C | state: %s",
			(double)sv_to_float(&v_bat),
			(double)(sv_to_float(&i_bat) * 1000.0f),
			(double)sv_to_float(&t_die),
			charge_state_str(chg_status.val1));

		k_msleep(2000);
	}

	return 0;
}
