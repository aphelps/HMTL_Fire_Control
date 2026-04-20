# HMTL Fire Control — ESP32 Touch Controller PCB Production Guide

Custom ESP32 PCB for the Wickerman touch controller. Functionally equivalent to an
ESP32-WROOM-32E DevKitC with HMTL-specific peripherals added.

---

## Board Overview

| Feature | Implementation |
|---------|---------------|
| MCU | ESP32-WROOM-32E-N8 (8 MB flash) |
| USB | USB-C via CP2102N (programming + 5V power) |
| RS485 | SP3485EN-L/TR, GPIO16/17/18 (UART2) |
| Touch | MPR121QR2, 12-channel capacitive, I2C 0x5A |
| LCD | PCF8574AT I2C expander on-board (I2C 0x27), connects to HD44780 LCD |
| LEDs | WS2801 (2-wire) via 74AHCT125D level shifter, GPIO13/HSPI-MOSI (DATA) + GPIO14/HSPI-SCLK (CLOCK) |
| Switches | 4× active-LOW rocker switches, GPIO26/27/32/33 |
| Power | AMS1117-3.3 LDO from 5V USB or J1 screw terminal |
| ESD | USBLC6-2SC6 on USB D+/D−; 500 mA PPTC polyfuse on VBUS |
| Connectors | All external I/O via through-hole KF301 5.0 mm screw terminals |

---

## GPIO Assignment

| GPIO | Function | Notes |
|------|----------|-------|
| 0 | BOOT button | Strapping pin — 10 kΩ pull-up to 3.3V |
| 2 | Status LED | Strapping pin — 10 kΩ pull-up + LED + 330 Ω to 3.3V |
| 4 | MPR121 IRQ | Interrupt-capable input |
| 16 | RS485 RX (UART2) | Serial2 |
| 17 | RS485 TX (UART2) | Serial2 |
| 18 | RS485 DE/RE enable | High = transmit, Low = receive |
| 21 | SDA (I2C) | MPR121 + PCF8574AT shared bus |
| 13 | WS2801 DATA out (HSPI MOSI) | Level-shifted to 5V via 74AHCT125D |
| 14 | WS2801 CLOCK out (HSPI SCLK) | Level-shifted to 5V via 74AHCT125D |
| 22 | SCL (I2C) | 4.7 kΩ pull-up to 3.3V |
| 26 | Rocker switch 1 | 10 kΩ pull-up, active-LOW |
| 27 | Rocker switch 2 | 10 kΩ pull-up, active-LOW |
| 32 | Rocker switch 3 | 10 kΩ pull-up, active-LOW |
| 33 | Rocker switch 4 | 10 kΩ pull-up, active-LOW |
| 1 | UART0 TX → CP2102N RX | USB serial (programming + debug) |
| 3 | UART0 RX ← CP2102N TX | USB serial |
| EN | Reset button | 10 kΩ pull-up, 100 nF debounce to GND |

**Do not use:** GPIO 6–11 (connected to SPI flash internally), GPIO 12 (flash voltage
strapping — leave floating to select 3.3V).

---

## Connector Map

All connectors are through-hole with 5.0 mm pitch — compatible with KF301 screw terminals
or direct wire soldering. Hole diameter 1.0 mm accepts 16–24 AWG wire.

| Connector | Signals | Type | Notes |
|-----------|---------|------|-------|
| J1 — Power in | 5V, GND | KF301-2P | Alternate 5V input; USB-C (J8) also powers board |
| J2 — RS485 | A, B, GND | KF301-3P | Connect to RS485 bus; 120 Ω termination jumper on-board |
| J3 — LCD | SDA, SCL, 3.3V, GND | KF301-4P | Connects directly to I2C LCD backpack |
| J4 — WS2801 LEDs | DATA, CLOCK, 5V, GND | KF301-4P | Both signals level-shifted to 5V via 74AHCT125D |
| J5 — Switches | SW1, SW2, SW3, SW4, GND | KF301-5P | Active-LOW; pull-ups on board |
| J6 — ELE0–5 | ELE0, ELE1, ELE2, ELE3, ELE4, ELE5, GND | KF301-7P | MPR121 touch electrodes |
| J7 — ELE6–11 | ELE6, ELE7, ELE8, ELE9, ELE10, ELE11, GND | KF301-7P | MPR121 touch electrodes |
| J8 — USB-C | Programming / 5V power | On-board | Connects to CP2102N |

**Screw terminals:** KF301 series accepts wire directly and can be removed/replaced.
For permanent connections the same holes accept 2.54 mm or 5.0 mm pin headers.

---

## Bill of Materials

### ICs and modules

| Component | LCSC# | DigiKey | Qty | Role |
|-----------|-------|---------|-----|------|
| ESP32-WROOM-32E-N8 | C701342 | [link](https://www.digikey.com/en/products/detail/espressif-systems/ESP32-WROOM-32E-N8/13159522) | 1 | Main MCU |
| MPR121QR2 | C91322 | [link](https://www.digikey.com/en/products/detail/nxp-usa-inc/MPR121QR2/2186527) | 1 | 12-ch capacitive touch, QFN-20 |
| CP2102N-A02-GQFN28R | C964632 | [link](https://www.digikey.com/en/products/detail/silicon-labs/CP2102N-A02-GQFN28R/9863480) | 1 | USB-serial bridge, QFN-28 |
| SP3485EN-L/TR | C8963 | Mouser 701-SP3485EN-LTR | 1 | RS485 transceiver, SOIC-8 |
| AMS1117-3.3 | C6186 | — | 1 | 3.3V LDO, SOT-223 |
| PCF8574AT | C398075 | — | 1 | I2C LCD expander, SOIC-16 |
| 74AHCT125D | C2595 | — | 1 | 3.3V→5V level shifter, SOIC-14 |
| USBLC6-2SC6 | C12044 | Mouser 511-USBLC6-2SC6 | 1 | USB ESD protection, SOT-23-6 |

**JLCPCB Basic parts** (no extended feeder fee): AMS1117-3.3, SP3485EN-L/TR, 74AHCT125D,
all passives listed below.

**JLCPCB Extended parts** (+$3 setup per type): ESP32-WROOM-32E, MPR121QR2, CP2102N,
PCF8574AT, USBLC6-2SC6.

### Passives and connectors

| Component | LCSC# | Value | Qty | Role |
|-----------|-------|-------|-----|------|
| 100 nF 0402 cap | C14663 | 100 nF | 12 | Decoupling |
| 10 µF 0805 cap | C15850 | 10 µF | 3 | Bulk decoupling |
| 100 µF electrolytic | C16133 | 100 µF 6.3V | 1 | 3.3V rail bulk cap |
| 10 kΩ 0603 resistor | C25804 | 10 kΩ | 8 | Pull-ups |
| 75 kΩ 0603 resistor | C23233 | 75 kΩ | 1 | MPR121 REXT |
| 120 Ω 0603 resistor | C23422 | 120 Ω | 1 | RS485 termination (install on last node) |
| 680 Ω 0603 resistor | C23171 | 680 Ω | 2 | RS485 bias (A→3.3V, B→GND) |
| 330 Ω 0603 resistor | C23137 | 330 Ω | 1 | Status LED current limit |
| 27 Ω 0402 resistor | C13389 | 27 Ω | 2 | USB D+/D− series |
| 4.7 kΩ 0603 resistor | C23162 | 4.7 kΩ | 2 | I2C pull-ups (SDA, SCL) |
| 500 mA PPTC 1206 | C16188 | 500 mA | 1 | VBUS polyfuse |
| USB-C connector (SMD) | C165948 | — | 1 | J8 |
| KF301-2P screw terminal | C8440 | 5.0 mm | 1 | J1 power |
| KF301-3P screw terminal | C8439 | 5.0 mm | 1 | J2 RS485 |
| KF301-4P screw terminal | — | 5.0 mm | 2 | J3 LCD, J4 WS2801 LEDs |
| KF301-5P screw terminal | — | 5.0 mm | 1 | J5 switches |
| KF301-7P screw terminal | — | 5.0 mm | 2 | J6, J7 MPR121 electrodes |

KF301 terminals are available as break-apart strips on Amazon ("KF301 5mm pitch PCB screw
terminal"). LCSC stocks the 2P and 3P individually; longer ones can be sourced from Amazon
or AliExpress.

### Estimated per-board cost (qty 10)

| Category | Cost |
|----------|------|
| ESP32-WROOM-32E-N8 | $3.50 |
| MPR121QR2 | $3.00 |
| CP2102N | $1.50 |
| PCF8574AT | $0.40 |
| USBLC6-2SC6 + polyfuse | $0.20 |
| SP3485EN-L/TR | $0.15 |
| 74AHCT125D | $0.20 |
| AMS1117-3.3 | $0.12 |
| USB-C connector | $0.15 |
| Passives (~40 pcs) | $0.60 |
| Screw terminals (~8 pcs) | $0.80 |
| **Total parts/board** | **~$10.60** |

JLCPCB PCBA all-in (PCB + SMT assembly + extended part fees amortized at qty 10):
**~$16–17 per board**.

---

## Obtaining the PCB

### Option A — Bare PCB (you assemble)

1. Download `hardware/gerbers/gerbers.zip` from this repo
2. Go to [jlcpcb.com/quote](https://jlcpcb.com/quote)
3. Upload the zip; set: 2-layer, 1.6 mm FR4, HASL, green solder mask, qty 5
4. Cost: ~$2–4 + shipping (~$15–25 standard to US, ~$8 DHL)
5. Lead time: 2–5 day production + 7–20 days standard / 3–5 days DHL

Alternative fabs: [PCBWay](https://www.pcbway.com) (comparable pricing, PCBA available),
[OSH Park](https://oshpark.com) (US-based, purple boards, 3 minimum, faster domestic shipping),
[MacroFab](https://macrofab.com) (US PCBA, higher cost, good for small qty).

### Option B — PCBA (SMT pre-assembled, you add screw terminals)

1. Download `hardware/gerbers/gerbers.zip`, `hardware/jlcpcb/bom.csv`, `hardware/jlcpcb/cpl.csv`
2. Go to [jlcpcb.com/smt-assembly](https://jlcpcb.com/smt-assembly)
3. Upload Gerbers; enable SMT Assembly; upload BOM and CPL files
4. Review component placements in the 2D viewer — check QFN rotation corrections
5. Estimated cost at qty 10: **~$16–17/board all-in**
6. After delivery: hand-solder the KF301 screw terminals (THT, not in PCBA)

**Note:** The `hardware/jlcpcb/rotation_corrections.json` file (generated by kicad-jlcpcb-tools)
contains CPL rotation offsets for QFN packages that differ between KiCad and JLCPCB's pick-and-place
coordinate system. This file must be committed and re-applied any time you regenerate the CPL.

---

## Hand-Assembly Guide (prototype phase)

Use this procedure when assembling bare PCBs by hand.

### Tools required

- Temperature-controlled soldering iron (Hakko FX-888D or equivalent)
  - Leaded solder: 300°C; lead-free: 350°C
- Hot air station (858D or equivalent) — required for MPR121 QFN-20 and CP2102N QFN-28
- Solder paste (Sn63Pb37 for prototyping)
- No-clean gel flux, solder wick, IPA 90%+
- Microscope or 10× loupe — essential for QFN inspection
- Multimeter with continuity mode
- PCB vise or helping hands

### Soldering order

Solder small/hot-sensitive parts first while the iron tip and paste are fresh:

1. **QFN-20 MPR121** and **QFN-28 CP2102N** — apply paste, reflow with hot air, inspect under loupe for bridges
2. **SOT-23-6 USBLC6-2SC6** — tiny, do early
3. **SOT-223 AMS1117-3.3**
4. **SOIC-8 SP3485**, **SOIC-14 74AHCT125D**, **SOIC-16 PCF8574AT**
5. **ESP32-WROOM-32E-N8** — tin the castellated pads on the PCB first, then drag-solder the module; inspect all pads
6. **0603 passives** (resistors and caps)
7. **1206 polyfuse**, **USB-C connector**
8. **THT screw terminals** (KF301) — solder from below

### Pre-power-on checklist

Do these checks before applying power for the first time:

1. Loupe inspection: no solder bridges on QFN pads, no lifted ESP32 castellations
2. Continuity check: 3.3V rail to GND — must NOT be short; 5V (VBUS) to GND — must NOT be short
3. Apply 5V from bench supply set to 200 mA current limit; measure 3.3V rail
4. Connect USB-C — CP2102N should enumerate as a serial port on your computer
5. I2C scan (`i2c_scanner.ino`) — expect 0x5A (MPR121) and 0x27 (PCF8574AT)
6. RS485 loopback: jumper A to B, confirm echo in serial monitor
7. Flash HMTL firmware and run Python test suite (see below)

---

## KiCad Design Notes

These notes apply when creating or modifying the schematic/layout in KiCad.

### Reference designs

- **ESP32-WROOM-32E** symbol + footprint: [espressif/kicad-libraries](https://github.com/espressif/kicad-libraries)
- **DevKitC schematic** (reference for LDO, ESD, buttons): [esp-dev-kits/esp32-devkitc](https://github.com/espressif/esp-dev-kits/tree/master/esp32-devkitc)
- **RS485 circuit reference**: [Xinyuan-LilyGO/T-CAN485](https://github.com/Xinyuan-LilyGO/T-CAN485)
- **MPR121 KiCad symbol**: [alexchow/MPR121-KiCad](https://github.com/alexchow/MPR121-KiCad); footprint: `Package_DFN_QFN:QFN-20-1EP_3x3mm_P0.5mm_EP1.5x1.5mm`
- **CP2102N layout guide**: Silicon Labs AN721
- **Full ESP32+RS485 reference board** (KiCad 6, Apache): [mcci-catena/Model4916](https://github.com/mcci-catena/Model4916)
- **JLCPCB tools plugin**: [Bouni/kicad-jlcpcb-tools](https://github.com/Bouni/kicad-jlcpcb-tools) — install via KiCad Plugin Manager; generates BOM + CPL in JLCPCB format

### Key schematic notes

- **RS485**: Tie DE and RE together to GPIO 18. Place 120 Ω termination resistor with a 2-pad jumper so it can be omitted on non-terminal nodes. Place 680 Ω bias resistors: A→3.3V, B→GND.
- **MPR121**: Pull ADDR to GND (I2C address 0x5A). REXT = 75 kΩ to GND. 100 nF + 10 µF on VDD pin. Route all 12 ELE traces to J6/J7 with no copper pour nearby (capacitive noise).
- **PCF8574AT**: Address pins A0–A2 to GND via 0 Ω resistors (change address by replacing). Shared I2C bus with MPR121.
- **WS2801 level shift**: Two channels of 74AHCT125D (one IC has 4 channels; two are used). GPIO13/HSPI-MOSI (DATA) → J4 DATA (5V); GPIO14/HSPI-SCLK (CLOCK) → J4 CLOCK (5V). Using HSPI pins allows FastLED to use hardware SPI. Bypass cap on VCC.
- **Strapping pins**: GPIO 0 — 10 kΩ pull-up + tact switch to GND (BOOT). EN — 10 kΩ pull-up + 100 nF to GND + tact switch to GND (RESET). GPIO 2 — 10 kΩ pull-up. GPIO 15 — 10 kΩ pull-up. GPIO 12 — leave floating.
- **Through-hole footprints**: Use 1.0 mm drill / 2.0 mm annular ring at 5.0 mm pitch. KiCad footprint: `Connector_PinArray:PinArray_1xNN_Pitch5.00mm` or create a custom footprint. These holes accept KF301 tabs or bare wire.

### JLCPCB Gerber export workflow

1. Assign LCSC C-numbers to all symbols (field name: `LCSC`)
2. Use kicad-jlcpcb-tools plugin: Tools → Generate JLCPCB fabrication files
3. This creates `hardware/jlcpcb/bom.csv` and `hardware/jlcpcb/cpl.csv`
4. Export Gerbers: File → Fabrication Outputs → Gerbers (use JLCPCB preset)
5. Zip Gerbers → `hardware/gerbers/gerbers.zip`
6. **Rotation check**: QFN and SOIC packages often differ 90° or 180° from KiCad's orientation. The plugin saves corrections to `hardware/jlcpcb/rotation_corrections.json` — commit this file so corrections survive regeneration.
7. Verify placement in JLCPCB's 2D viewer before ordering.

---

## Flashing Firmware

```bash
# Install PlatformIO
pip install platformio

# Get the firmware
git clone <repo-url> HMTL_Fire_Control
cd HMTL_Fire_Control
git checkout touchcontroller_esp32

# Build and flash (replace PORT with your device's serial port)
cd platformio/HMTL_Fire_Control_Wickerman
pio run -e touchcontroller_esp32 --target upload --upload-port /dev/cu.usbmodem-XXXX

# Monitor serial output
pio device monitor --port /dev/cu.usbmodem-XXXX --baud 115200
```

Find your port on macOS: `ls /dev/cu.usb*`

---

## EEPROM Configuration

After flashing for the first time, the EEPROM config must be written to tell the firmware
which pins are used for each peripheral. Use `HMTLConfig.py` from the HMTL python tools:

```bash
cd HMTL/python
python bin/HMTLConfig.py --port /dev/cu.usbmodem-XXXX --write configs/HMTL_Fire_Control_ESP32.json
```

The config JSON must specify:

| Module | Parameter | Value |
|--------|-----------|-------|
| rs485 | recvpin | 16 |
| rs485 | xmitpin | 17 |
| rs485 | enablepin | 18 |
| mpr121 | irqpin | 4 |
| pixels | datapin | 13 |
| pixels | clockpin | 14 |

---

## Verification Checklist

Run these checks after a fresh assembly and flash:

- [ ] Power via USB-C; measure 3.3V at J1 or LDO output
- [ ] I2C scan: MPR121 at 0x5A, PCF8574AT at 0x27
- [ ] Flash `touchcontroller_esp32` firmware; device prints `ready` on serial
- [ ] Python unit tests pass (no hardware needed):
  ```bash
  cd python && python3 -m pytest tests/unit/ -v
  ```
- [ ] Python integration tests pass (device connected):
  ```bash
  cd python && python3 -m pytest tests/integration/ --touch-port /dev/cu.usbmodem-XXXX -v
  ```
- [ ] Touch each MPR121 electrode (ELE0–ELE11) — correct sensor index appears in serial output
- [ ] Toggle each rocker switch — debug line appears per switch
- [ ] LCD shows startup message then status screen
- [ ] RS485: connect to an HMTL_Module, send poll, confirm response
- [ ] WS2801 pixels illuminate in each control mode
- [ ] Run same Python integration test suite against ATmega target and compare results
