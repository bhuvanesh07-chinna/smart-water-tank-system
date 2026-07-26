# ESP32 Smart Overhead Tank Water Quality & Supply Monitor

## Folder Structure
```
smart_tank_monitor/
├── smart_tank_monitor.ino   <- main sketch (open this in Arduino IDE)
└── README.md                <- this file
```
> Arduino IDE rule: the `.ino` file name MUST match its parent folder name — already done here.

## Pin Connections

| Function              | ESP32 Pin | Component        |
|-----------------------|-----------|-------------------|
| Ultrasonic Trigger    | GPIO 5    | HC-SR04 TRIG      |
| Ultrasonic Echo       | GPIO 18   | HC-SR04 ECHO      |
| Water Quality Sensor  | GPIO 34   | Analog quality sensor OUT |
| Sump Level Sensor     | GPIO 35   | Analog sump level sensor OUT |
| Pump Relay            | GPIO 26   | Relay IN (drives pump) |
| Quality LED           | GPIO 27   | LED (+ resistor)  |
| Buzzer                | GPIO 25   | Active buzzer     |

Notes:
- HC-SR04 ECHO is 5V logic — use a voltage divider (or logic level shifter) before GPIO 18 to protect the ESP32.
- GPIO 34 and 35 are input-only (ADC1) pins — correct choice for analog sensors, no pull-up/down needed for output pins.

## Required Setup (Arduino IDE)
1. Install **ESP32 board package** via Boards Manager (`https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json` in Preferences → Additional Board URLs).
2. Tools → Board → select your ESP32 dev board (e.g. "ESP32 Dev Module").
3. Tools → Port → select the correct COM/tty port.
4. Open `smart_tank_monitor.ino` and click Upload.
5. Open Serial Monitor at **115200 baud** to see live readings.

## Configurable Parameters (top of file)
- `TANK_CAPACITY_LITRES`, `TANK_HEIGHT_CM`, `SENSOR_EMPTY_DISTANCE_CM` — calibrate to your tank's physical dimensions.
- `PUMP_START_LEVEL` (30%), `PUMP_STOP_LEVEL` (90%), `SUMP_EMPTY_LEVEL` (20%) — pump control thresholds.
- `QUALITY_MIN` / `QUALITY_MAX` (400–600) — acceptable water quality band.
- `REQUIRED_ABNORMAL_SAMPLES` (5) — consecutive bad readings before alarm triggers (rejects single spikes).

## Behavior Summary
- Pump turns **ON** when tank level ≤ 30%, **OFF** when ≥ 90% (hysteresis in between keeps prior state).
- Pump is force-stopped if sump level ≤ 20% (dry-run protection) or tank sensor gives an invalid reading.
- Water quality is smoothed with a 5-sample moving average; only 5 consecutive out-of-range readings raise the alarm (LED + buzzer), filtering out single-sample spikes.
- A rising tank level while the pump is OFF is logged as a possible tanker/external fill event.

## Demo Video

[![Watch the demo](demo_video.mp4)

Click the image above to watch the demo, or check `docs/demo.mp4` in this repo.
