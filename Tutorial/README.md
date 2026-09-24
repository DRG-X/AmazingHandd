# Amazing Hand: software tutorial, from zero to a working hand

This tutorial assumes the **mechanical assembly is done** and you have the
electronics on your desk. It takes you, in order, from an empty folder to:

1. a hand that plays the gesture demo (Python or Arduino),
2. a reusable Python driver plus an interactive command line to pose the hand,
3. *(advanced)* a hand that copies your own hand live through a webcam
   (MediaPipe → MuJoCo inverse kinematics → Rust motor controller, all glued together by dora-rs).

You said you want to **understand and type every file yourself**. Each chapter therefore:

* explains the idea first,
* gives you the complete file to type (the repo's code, cleaned up and commented, with the
  few repo bugs fixed and flagged),
* explains the file block by block,
* ends with a **✅ Checkpoint** so you know it works before you move on.

Every Python/Rust file in this tutorial was run before it was written down here: against a
simulated servo bus, a headless MuJoCo, and a real `dora` dataflow with a fake serial port.
None of it has been tested on physical servos, so go slowly the first time you power the hand.
The Arduino sketches are the repo's sketches plus comments and one typo fix. I could not compile
them here.

---

## Which path should I follow?

The repo supports two ways of driving the 8 servos:

| | **Path A: PC + Python** (recommended) | **Path B: Arduino** |
|---|---|---|
| Interface board | Waveshare *Bus Servo Adapter* (or any Feetech/SCS USB bus adapter) | Arduino (Mega recommended, Uno works) + Feetech *TTLinker* |
| Language | Python + `rustypot` library | C++ + `SCServo` library |
| Needed for the advanced webcam demo | **Yes** | No (the advanced demo needs a PC-side USB adapter) |
| Chapters | 1 → 8, then 10 → 14 | 1, 2, 9 (+ 5/6 for concepts) |

If you have the Waveshare adapter, follow Path A. Read chapter 9 only if you want the Arduino version too.

---

## Table of contents

| # | Chapter | You will write |
|---|---|---|
| 01 | [How the Amazing Hand works (software view)](01_How_It_Works.md) | nothing, just theory (read it!) |
| 02 | [Hardware setup, wiring and safety](02_Hardware_Setup.md) | nothing |
| 03 | [PC setup: Python, virtual env, serial port](03_PC_Setup.md) | project folder, `requirements.txt` |
| 04 | [Under the hood: the Feetech serial protocol](04_Servo_Protocol.md) | `raw_protocol.py` (optional but eye-opening) |
| 05 | [First contact: scan the bus and set the IDs](05_Scan_And_IDs.md) | `scan_bus.py`, `change_id.py` |
| 06 | [Calibration: the middle positions](06_Calibration.md) | `finger_middle_pos.py`, `finger_test.py`, `read_positions.py` |
| 07 | [The full hand demo](07_Hand_Demo.md) | `hand_demo.py` |
| 08 | [Level up: a hand library, an interactive CLI, two hands](08_Library_CLI_TwoHands.md) | `amazing_hand.py`, `hand_cli.py`, `hand_demo_both.py` |
| 09 | [Path B: Arduino + TTLinker](09_Arduino.md) | 2 Arduino sketches |
| 10 | [Advanced demo 1/4: architecture, tools, dora-rs](10_Advanced_Setup.md) | `Demo/` skeleton, `Cargo.toml`, `pyproject.toml`s |
| 11 | [Advanced demo 2/4: AHControl, the Rust motor node](11_AHControl_Rust.md) | `main.rs` + 4 tools + TOML config |
| 12 | [Advanced demo 3/4: AHSimulation, MuJoCo + inverse kinematics](12_AHSimulation.md) | `hand_sim.py`, `mj_mink_*.py`, `finger_angle_control.py` |
| 13 | [Advanced demo 4/4: HandTracking + running everything](13_HandTracking_And_Run.md) | `main.py`, the dataflow YAMLs |
| 14 | [Troubleshooting & next steps](14_Troubleshooting.md) | — |

---

## What your project will look like at the end

```
my_amazing_hand/
├── requirements.txt
├── python/                      # Path A (chapters 3-8)
│   ├── raw_protocol.py          # ch 4  - hand-made protocol driver (learning)
│   ├── scan_bus.py              # ch 5  - find servos
│   ├── change_id.py             # ch 5  - set IDs
│   ├── finger_middle_pos.py     # ch 6  - servos to middle (horn mounting)
│   ├── finger_test.py           # ch 6  - open/close one finger (calibration)
│   ├── read_positions.py        # ch 6  - live angles, torque off
│   ├── hand_demo.py             # ch 7  - the gesture demo
│   ├── amazing_hand.py          # ch 8  - reusable driver class
│   ├── hand_cli.py              # ch 8  - interactive keyboard control
│   └── hand_demo_both.py        # ch 8  - two hands on one bus (optional)
├── arduino/                     # Path B (chapter 9)
│   ├── Amazing_Hand_Finger_Test/Amazing_Hand_Finger_Test.ino
│   └── Amazing_Hand_Demo/Amazing_Hand_Demo.ino
└── Demo/                        # advanced demo (chapters 10-13)
    ├── Cargo.toml
    ├── dataflow_angle_simu.yml
    ├── dataflow_tracking_simu.yml
    ├── dataflow_tracking_real.yml
    ├── dataflow_tracking_real_2hands.yml
    ├── AHControl/               # Rust: config + motor node + tools
    ├── AHSimulation/            # Python: MuJoCo model + IK node
    └── HandTracking/            # Python: webcam + MediaPipe node
```

## Map: original repo file → your file

| Repo file | Your file | Chapter |
|---|---|---|
| `PythonExample/AmazingHand_Hand_FingerMiddlePos.py` | `python/finger_middle_pos.py` | 6 |
| `PythonExample/AmazingHand_FingerTest.py` | `python/finger_test.py` | 6 |
| `PythonExample/AmazingHand_Demo.py` | `python/hand_demo.py` | 7 |
| `PythonExample/AmazingHand_Demo_Both.py` | `python/hand_demo_both.py` | 8 |
| `ArduinoExample/Amazing_Hand-Finger_Test.ino` | `arduino/Amazing_Hand_Finger_Test/…ino` | 9 |
| `ArduinoExample/Amazing_Hand_Demo.ino` | `arduino/Amazing_Hand_Demo/…ino` | 9 |
| `Demo/Cargo.toml`, `Demo/AHControl/**` | same paths | 10, 11 |
| `Demo/AHSimulation/**` | same paths (`hand_sim.py` is new: it merges the shared code of the two `mj_mink_*` files) | 10, 12 |
| `Demo/HandTracking/**` | same paths | 10, 13 |
| `Demo/*.yml` | same paths | 13 |
| *(new)* | `raw_protocol.py`, `scan_bus.py`, `change_id.py`, `read_positions.py`, `amazing_hand.py`, `hand_cli.py` | 4, 5, 6, 8 |

Files you should **copy, not type**: the MuJoCo model (`Demo/AHSimulation/AHSimulation/AH_Right|AH_Left/`,
about 750 lines of XML generated from CAD, plus binary STL meshes). Chapter 12 explains what is in them.

Start here → [Chapter 01](01_How_It_Works.md)
