# KPAW: Wireless Dog Agility Timing and Jump-Height System

KPAW is a low-cost prototype for dog agility training. One gate unit times the dog, tells you which way it went (through the gate, or around the left or right post), and sets the jump bar to the right height. A browser app turns the data into run records and analytics.

This repository holds the firmware, web app and mechanical CAD files from my BSc Electrical and Computer Engineering final-year project (EEE4022S, University of Cape Town, 2026).

- **Student:** Khaarendiwe Mulaudzi
- **Supervisor:** Jarryd Son
- **Live web app:** <https://kpaw.lovable.app> (open in Chrome or Edge)
- **Full report:** To be uploaded.

## Why

Agility competitions give one course time, so a trainer can't see where a dog gains or loses time. Commercial wireless timing sets are expensive (the ALGE WTN-Set 1 is listed at about R48,000). The parts for this prototype cost about **R1,138**, inside the R2,000 project budget (3D printing, resistors, transistors, wires and breadboard not included).

## How it works

| Subsystem | What it does | Hardware |
|---|---|---|
| Infrared timing gate | Detects a dog crossing the gate with four 38 kHz modulated beams at different heights. A crossing needs two or more beams blocked, then a 1 s cooldown. | 4 × TSAL6100 emitters, HIR-138LN receiver |
| Ultrasonic path mapping | Detects passes around the left and right posts. | 2 × HC-SR04 |
| Jump-height mechanism | Sets the bar between 15 and 60 cm using a stepper-driven lead screw. | NEMA 17, DRV8825, TR8×2 lead screw |
| Controller | Two ESP32 boards. Events are timestamped on the gate, so Bluetooth delay does not affect measured intervals. | 2 × ESP32 dev boards |
| Web app | Connects over Bluetooth Low Energy (Web Bluetooth), handles dog profiles, run recording, path map, analytics and jump-height selection. | Chrome or Edge, HTTPS or localhost |

## Repository layout

```
.
├── CODE/    Firmware (Arduino IDE / ESP32) and web app source
├── CAD Files/     Autodesk Inventor files for the gate frame and 3D-printed parts
└── README.md
```

### `code/`

- `firmware/`: ESP32 firmware (Arduino IDE). Includes IR transmitter code, IR receiver code, ultrasonic sensing, stepper control and the BLE command interface.

**Firmware dependencies** (Arduino IDE, ESP32 board package):
- `AccelStepper`
- `Preferences` (included with the ESP32 core)
- `RTC_DS3231`
- ESP32 BLE support (included with the ESP32 core)

**Key firmware settings**

| Setting | Value |
|---|---|
| Stepper driver | DRV8825, full-step, STEP on GPIO26, DIR on GPIO27 |
| Max speed and acceleration | 1400 steps/s, 700 steps/s² |
| Calibration | 993.6 steps/cm (15 cm is the zero reference, set by hand) |
| Travel limits | 15–60 cm, validated in both the app and the firmware |
| IR receiver input | GPIO15 on board B |
| Status LEDs | GPIO5, 18, 19 |

**Flashing the firmware**
1. Install the Arduino IDE and the ESP32 board package.
2. Install the libraries listed above.
3. Open the sketch for each board, select the matching board and port, and upload. Board A drives the IR emitters and ultrasonic sensor 1. Board B carries the IR receiver, stepper driver, ultrasonic sensor 2 and the BLE link.
4. Before the first move, set the carriage at the 15 cm mark and send the zero command over the serial monitor.

**BLE text commands**

| App to ESP32 | Meaning |
|---|---|
| `go X` | Move the bar to height X (cm) |
| `stop` | Stop the motor |
| `position` | Request current position |
| `sensors` | Request sensor states |
| `run start` / `run end` | Begin or end a run |

Gate events come back as text lines such as `GATE <us> <beams>`, `US <ms> <left> <right>` and `RUN ended <ms>`. Long messages are sent in 20-byte chunks.

**Web app**

Use the hosted version, or run the source locally (Web Bluetooth needs HTTPS or `localhost`). The app has a preview mode that simulates the motor and replays runs when no device is connected.

### `cad/`

Autodesk Inventor files for the mechanical assembly:

- 1000 mm vertical mast and 500 mm crossbar from 25 × 25 mm square bar
- Lead-screw lifting assembly with two guide rails and a carriage
- 3D-printed parts: base, bearing housings and bushings, carriage and carriage bushings, bar holder, top holder, and the ultrasonic sensor enclosure.

The assembly figure in the report (Figure 3.5) labels all 16 parts.

## Test results

| Test | Result |
|---|---|
| IR gate, 10 crossings at 1 s spacing | 10 of 10 detected |
| IR gate, 20 crossings at 0.5 s spacing | 10 of 20 detected (1 s cooldown) |
| IR range | Reliable at 50 cm. False or missed detections at 60–120 cm (5 of 10 detected at 120 cm). |
| Timestamps against video | 0.3–0.5 s difference |
| Ultrasonic path sequences | 9 of 9 classified correctly |
| Jump-bar positioning | Within 3 mm at 20, 30, 45 and 60 cm |
| Saved runs | 10 of 10 persisted after restart |

## Limitations

- Tested on one side of the gate, with a person standing in for the dog. No dog trials or usability study.
- The IR gate is not reliable across the full 1.2 m gate width. Emitters run at about 5 mA, which probably limits range.
- The jump-bar mechanism is open-loop (no limit switch or encoder) and stops at 60 cm, below the top of the app's Large range.
- Time to jump currently uses the first detected event, not the first through-gate pass.
- No fault response if Bluetooth disconnects while the bar is moving.
- One gate only, so no split times yet.
- Web Bluetooth does not work in Safari or on iPhones. Profiles and runs are stored in the browser.

## Future work

1. Stronger IR emitters, state-based re-arming and a video-synchronised timing test.
2. Fix time to jump, add an emergency stop and support every height category.
3. Add more gates for split times, then run supervised dog and usability trials.

## Acknowledgements

Claude, ChatGPT and Lovable were used for design support, troubleshooting and software development, as declared in the project report. 

## Licence

All rights reserved.
