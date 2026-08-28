/*
 * Nordic_Solar — custom BLE GATT service: battery + solar/charge status
 */
#ifndef SOLAR_SERVICE_H_
#define SOLAR_SERVICE_H_

#include <zephyr/types.h>
#include <stdbool.h>

/* 128-bit vendor UUIDs (private, randomly assigned for this project).
 * Service : 5c1b0000-9f2a-4b6d-8e3f-1a2b3c4d5e6f
 * Status  : 5c1b0001-9f2a-4b6d-8e3f-1a2b3c4d5e6f
 */
#define BT_UUID_SOLAR_SVC_VAL \
	BT_UUID_128_ENCODE(0x5c1b0000, 0x9f2a, 0x4b6d, 0x8e3f, 0x1a2b3c4d5e6f)
#define BT_UUID_SOLAR_STATUS_VAL \
	BT_UUID_128_ENCODE(0x5c1b0001, 0x9f2a, 0x4b6d, 0x8e3f, 0x1a2b3c4d5e6f)

/* Charge state values matching the nPM1300 status decode. */
enum solar_charge_state {
	SOLAR_CHG_NO_BATTERY = 0,
	SOLAR_CHG_IDLE       = 1,
	SOLAR_CHG_TRICKLE    = 2,
	SOLAR_CHG_CC         = 3,
	SOLAR_CHG_CV         = 4,
	SOLAR_CHG_COMPLETE   = 5,
};

/* Packed payload of the status characteristic (7 bytes, little-endian). */
struct solar_status {
	uint16_t vbat_mv;      /* battery voltage, millivolts */
	int16_t  ibat_ma;      /* battery current, milliamps (+ = charging) */
	uint8_t  charge_state; /* enum solar_charge_state */
	uint8_t  vbus_present; /* 1 = USB/solar input present */
	uint8_t  battery_pct;  /* 0..100 estimated state of charge */
} __packed;

/* Push a new snapshot into the characteristic and notify subscribers. */
void solar_service_update(const struct solar_status *s);

#endif /* SOLAR_SERVICE_H_ */
