/*
 * Nordic_Solar — Stage 1: LED blinky (nPM1300 LED0)
 *
 * Wiring: VSYS -> R14 (560R) -> LED1 anode -> cathode -> nPM1300 pin 25 (LED0).
 * I2C: SCL P1.03 / SDA P1.04, 4.7k (R12/R13) to VDDIO, nPM1300 @ 0x6B.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>

#include <hal/nrf_twim.h>

LOG_MODULE_REGISTER(blinky, LOG_LEVEL_INF);

#define PMIC_LED_NODE   DT_NODELABEL(npm1300_leds)
#define PMIC_I2C_NODE   DT_NODELABEL(i2c22)
#define PMIC_I2C_ADDR   0x6B
#define STATUS_LED_IDX  0
#define BLINK_PERIOD_MS 1000
#define WIGGLE_MS       2000

static void probe_pmic(const struct device *i2c)
{
	uint8_t dummy = 0;
	int ret;

	if (!device_is_ready(i2c)) {
		LOG_ERR("i2c22 is not ready");
		return;
	}

	ret = i2c_write(i2c, &dummy, 0, PMIC_I2C_ADDR);
	if (ret == 0) {
		LOG_INF("nPM1300 ACKed at 0x%02x", PMIC_I2C_ADDR);
	} else {
		LOG_ERR("nPM1300 no ACK at 0x%02x (err %d)", PMIC_I2C_ADDR, ret);
	}
}

/*
 * Disconnect TWIM22 and drive the I2C pads as GPIO so a DMM on the *I2C*
 * side of R12 (SCL) / R13 (SDA) can see 3 V vs 0 V. Idle pull-up is 3 V
 * on both sides of those resistors — that is normal and does not prove
 * the SoC is toggling the net.
 */
static void pin_wiggle(void)
{
	const struct device *gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));

	if (!device_is_ready(gpio1)) {
		LOG_ERR("gpio1 not ready — cannot wiggle I2C pins");
		return;
	}

	nrf_twim_disable(NRF_TWIM22);
	nrf_twim_pins_set(NRF_TWIM22, 0xFFFFFFFF, 0xFFFFFFFF);

	gpio_pin_configure(gpio1, 3, GPIO_OUTPUT_HIGH); /* P1.03 SCL / R12 */
	gpio_pin_configure(gpio1, 4, GPIO_OUTPUT_HIGH); /* P1.04 SDA / R13 */

	LOG_INF("GPIO wiggle on I2C pads (2 s steps). Meter the I2C side of R12/R13:");
	LOG_INF("  R12 = SCL = P1.03");
	LOG_INF("  R13 = SDA = P1.04");

	while (1) {
		LOG_INF("both HIGH — R12 and R13 I2C side ~3 V");
		gpio_pin_set(gpio1, 3, 1);
		gpio_pin_set(gpio1, 4, 1);
		k_msleep(WIGGLE_MS);

		LOG_INF("SCL LOW  — R12 I2C side ~0 V, R13 stays ~3 V");
		gpio_pin_set(gpio1, 3, 0);
		gpio_pin_set(gpio1, 4, 1);
		k_msleep(WIGGLE_MS);

		LOG_INF("SDA LOW  — R13 I2C side ~0 V, R12 stays ~3 V");
		gpio_pin_set(gpio1, 3, 1);
		gpio_pin_set(gpio1, 4, 0);
		k_msleep(WIGGLE_MS);

		LOG_INF("both LOW — R12 and R13 I2C side ~0 V");
		gpio_pin_set(gpio1, 3, 0);
		gpio_pin_set(gpio1, 4, 0);
		k_msleep(WIGGLE_MS);
	}
}

int main(void)
{
	const struct device *leds = DEVICE_DT_GET(PMIC_LED_NODE);
	const struct device *i2c = DEVICE_DT_GET(PMIC_I2C_NODE);
	bool on = false;
	int ret;

	LOG_INF("Nordic_Solar bring-up — Stage 1: nPM1300 LED0 blinky");
	LOG_INF("i2c22 ready=%d  npm1300_leds ready=%d",
		device_is_ready(i2c), device_is_ready(leds));

	probe_pmic(i2c);

	if (!device_is_ready(leds)) {
		LOG_ERR("PMIC LED not ready — falling back to SCL/SDA GPIO wiggle");
		pin_wiggle();
		return -ENODEV;
	}

	LOG_INF("nPM1300 reached over I2C. Blinking LED0 at %d ms.", BLINK_PERIOD_MS);

	while (1) {
		on = !on;
		ret = on ? led_on(leds, STATUS_LED_IDX)
			 : led_off(leds, STATUS_LED_IDX);
		if (ret < 0) {
			LOG_ERR("led_%s failed (%d)", on ? "on" : "off", ret);
		} else {
			LOG_INF("LED0 %s", on ? "ON" : "off");
		}
		k_msleep(BLINK_PERIOD_MS / 2);
	}

	return 0;
}
