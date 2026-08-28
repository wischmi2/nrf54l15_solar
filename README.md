# Nordic_Solar — bring-up firmware

Staged nRF Connect SDK (Zephyr) firmware for the **nRF54L15 + nPM1300** solar
soil-moisture board (v2). Four small apps mirror your bring-up checklist so you
can flash one at a time and isolate faults before combining them:

| # | App | Proves |
|---|-----|--------|
| 1 | `01_led_blinky` | SoC → I²C → nPM1300 path (blinks the status LED via LED0) |
| 2 | `02_charger_enable` | Charger configured + enabled; live charge state |
| 3 | `03_solar_check` | Input (USB/solar) present + battery charging |
| 4 | `04_ble_battery_solar` | BLE peripheral exposing battery + solar status |

Target: `nrf54l15dk/nrf54l15/cpuapp` · SDK: nRF Connect SDK v3.3 · Console: RTT
(no UART is broken out — logs come back over the SWD/J-Link RTT channel).

---

## I²C pins (nPM1300)

Every app shares one overlay:
`<app>/boards/nrf54l15dk_nrf54l15_cpuapp.overlay` (source of truth: `common/`).

| Net | QFN pin | GPIO | TWIM |
|-----|---------|------|------|
| SCL | 4 | P1.03 | `&i2c22` |
| SDA | 5 | P1.04 | `&i2c22` |

`uart20` is disabled in that overlay so it does not steal P1.04 (the nRF54L15 DK board file uses it as UART TX). PMIC_INT is P0.00 (QFN pin 23) but is not enabled yet — that needs the nPM1300 GPIO used as INT.

If you make a custom Zephyr board definition, you can drop the overlay and describe the PMIC in the board `.dts` instead.

---

## Build & flash

Power the board (USB-C or battery) **before** attaching the debugger — J3 pin 1
is VTREF sense only and does not power the board. Then, per app:

```bash
cd 01_led_blinky
west build -b nrf54l15dk/nrf54l15/cpuapp
west flash
```

View logs over RTT:

```bash
JLinkRTTViewer            # GUI, or:
west rtt                  # if your setup supports it
```

Rebuild clean when switching apps: `west build -b nrf54l15dk/nrf54l15/cpuapp -p always`.

---

## Stage notes

**1 — LED blinky.** The only LED is on nPM1300 LED0, so a blink proves I²C to the
PMIC, not just a GPIO. LED off but I²C OK → check `led0-mode = "host"`. Device
not ready → I²C pins/instance or VDDIO (3.0 V) / PMIC ACK at 0x6B.

**2 — Charger.** Charge parameters are applied from the overlay (VTERM 4.2 V,
ICHG 200 mA, VBUS limit 500 mA) and `charging-enable` turns it on at boot. The
app prints the configured profile and the live state (TRICKLE → CC → CV →
COMPLETE) with battery voltage/current. Tune ICHG for your 400 mAh cell.

**3 — Solar check.** USB-C and solar are diode-OR'd into one VBUS node, so the
PMIC **cannot tell solar from USB electrically**. To confirm the panel: unplug
USB, then shade vs. expose the panel — VBUS should go absent → present with
charge current > 0. That transition is your proof the SPV1040T → D2 → CHG_IN
path works.

**4 — BLE.** Advertises as `Nordic_Solar`. Exposes the standard **Battery
Service** (level %) plus a custom service `5c1b0000-9f2a-4b6d-8e3f-1a2b3c4d5e6f`
whose single characteristic carries a packed struct: battery mV, current mA,
charge state, VBUS present, battery %. Connect with **nRF Connect for Mobile**,
enable notifications on the custom characteristic to watch it live. Battery % is
a simple voltage estimate — swap in the nRF Fuel Gauge library
(`CONFIG_NRF_FUEL_GAUGE`) for accurate SoC later.

---

## Safety gap to remember

The NTC pin is faked with a fixed 10 kΩ (no real thermistor), so the charger
reads a static ~25 °C and **never NTC-faults — but there is no real battery
temperature protection.** Fine on the bench; add a real NTC bonded to the cell
before any outdoor deployment.

---

## Verified against

nPM1300 driver bindings and APIs were checked against current Zephyr/NCS:
compatibles `nordic,npm1300` / `-charger` / `-regulator` / `-led`; sensor
channels `SENSOR_CHAN_NPM13XX_CHARGER_STATUS` and the gauge channels; Kconfig
`MFD_NPM13XX` / `LED_NPM13XX` / `NPM13XX_CHARGER`. If your exact SDK
revision still uses the older `npm1300`-spelled sensor header/Kconfig, change
`npm13xx` → `npm1300` in the `#include`, the `SENSOR_CHAN_NPM13XX_*` names, and
`CONFIG_NPM13XX_CHARGER` — the devicetree stays the same.
