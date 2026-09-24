# 01 · How the Amazing Hand works (software view)

Read this chapter carefully. Everything later (calibration, gestures, IK) is just
these ideas turned into code.

## 1.1 The big picture

```
 ┌──────────┐   USB    ┌──────────────────┐  3-wire bus (GND, +5V, DATA)  ┌────┐ ┌────┐     ┌────┐
 │ PC       │ ───────► │ Bus adapter      │ ─────────────────────────────►│ ID1│─│ ID2│ ... │ ID8│
 │ (Python) │          │ (Waveshare, or   │                               └────┘ └────┘     └────┘
 └──────────┘          │ Arduino+TTLinker)│◄── 5 V power supply (2-3 A)
                       └──────────────────┘
```

* The hand has **8 smart servos**, Feetech **SCS0009**, 2 per finger, 4 fingers
  (index, middle, ring, thumb).
* All 8 servos are **daisy-chained on one serial bus**. Each servo has an **ID**
  (1 to 253). A message on the bus starts with the ID of the servo it is for; only
  that servo executes it and answers.
* The bus is **half-duplex TTL serial at 1 000 000 baud**: one data wire, used for
  both directions, one direction at a time.
* The servo does the position control itself (its own PID loop). We only
  send **goal positions** (and a speed); we can also read back position, speed, load,
  voltage and temperature.

## 1.2 The SCS0009 servo in numbers

| Thing | Value |
|---|---|
| Position range | 0 … 1023 **raw steps** over **300°** |
| Resolution | 1 step = 300/1024 = **0.293°** (≈ 3.41 steps per degree) |
| Middle | raw **511** = 0 rad in `rustypot` |
| Factory ID | **1** (every new servo!) |
| Factory baudrate | 1 000 000 |
| Byte order of 16-bit registers | **big-endian** (high byte first). The STS series is little-endian, which is why the libraries treat them differently |
| Supply | 5 V recommended for this project |

Conversions you will use all the time:

```
raw     = 511 + degrees / 0.293          degrees = (raw - 511) * 0.293
radians = degrees * pi / 180             rustypot speed: rad/s   (1 rad/s ≈ 195.6 raw steps/s)
```

So the Arduino code (raw units) and the Python code (degrees → radians) describe the
same thing: `MiddlePos = 520` in Arduino is `MiddlePos = (520-511)*0.293 ≈ +2.6°` in Python.

## 1.3 The servo memory table (registers)

A servo is a small memory: you **read** and **write** bytes at addresses. The ones we use
(from `rustypot`'s SCS0009 definition and the Feetech FD software):

| Addr | Name | Size | Area | Meaning |
|---:|---|---:|---|---|
| 5 | ID | 1 | EEPROM | servo ID (persistent) |
| 6 | Baud rate | 1 | EEPROM | 0 = 1 000 000 |
| 9 / 11 | Min / Max angle limit | 2 | EEPROM | default 20 / 1003 raw (≈ ±144°) |
| 40 | Torque enable | 1 | RAM | **1 = holds position, 0 = limp** |
| 42 | Goal position | 2 | RAM | where to go (raw) |
| 44 | Goal time | 2 | RAM | alternative to speed (0 = unused) |
| 46 | Goal speed | 2 | RAM | steps/s |
| 48 | Lock | 1 | RAM | EEPROM write protection |
| 56 | Present position | 2 | RAM | where it is (raw) |
| 58 | Present speed | 2 | RAM | |
| 60 | Present load | 2 | RAM | effort, signed |
| 62 | Present voltage | 1 | RAM | unit 0.1 V |
| 63 | Present temperature | 1 | RAM | °C |
| 66 | Moving | 1 | RAM | 1 while moving |

EEPROM values survive power-off (ID, limits…). RAM values are reset at power-on (torque, goal…).

## 1.4 How one finger moves: two motors in parallel

![finger](../Demo/docs/finger.png)

Each finger has **motor 1** and **motor 2** side by side, each pushing a ball-joint rod.
The two rods drive the finger base through a gimbal:

* both horns turn **the same physical way** → the finger **bends/straightens** (flexion/extension)
* horns turn **opposite physical ways** → the finger **tilts sideways** (abduction/adduction)

Because the two servos are mounted **mirrored**, "same physical way" means **opposite
numeric signs**. That is the key to reading every gesture in the code:

| Motor angles `(motor1, motor2)` | Result |
|---|---|
| `( 90, -90)` | finger fully **closed** |
| `(-35,  35)` | finger **open**, straight |
| `(  0,   0)` | "middle" pose (the calibration reference) |
| `(-10,  80)` | open and tilted to one side ("nonono" right) |
| `(-80,  10)` | open and tilted to the other side ("nonono" left) |

You can turn this into two intuitive numbers:

```
flex = (motor1 - motor2) / 2      # + closes, - opens
abd  = (motor1 + motor2) / 2      # sideways tilt
motor1 = flex + abd
motor2 = abd - flex
```

Check: `(-10, 80)` → flex = -45 (open), abd = +35 (tilted). We will use this in chapter 8.

**Left hand = mirror:** for the same gesture, a left hand uses `(-motor2, -motor1)`:
the flexion is the same and the abduction is reversed. For example, the right-hand
victory index `(-15, 65)` becomes `(-65, 15)` on the left hand. The repo demos follow this rule.

## 1.5 Why calibration is needed

A servo horn sits on a splined shaft, so it can only be mounted at discrete angles: it
always ends up a few degrees away from the ideal. The 3D-printed parts and hand-adjusted
ball-joint rods add their own error. **Calibration** measures, for each of the 8 servos, the angle
`MiddlePos[i]` (in degrees) that has to be *added* so that "0°" really means the
reference pose. After that, every command is:

```
goal = MiddlePos[i] + gesture_angle
```

## 1.6 IDs: which servo is where

![hand](../Demo/docs/r_hand.png)

Right hand (and a standalone left hand, which uses the same IDs; the software mirrors the gestures):

| Finger | Motor 1 | Motor 2 | Name in the advanced demo |
|---|---|---|---|
| Index | 1 | 2 | `r_finger1` |
| Middle | 3 | 4 | `r_finger2` |
| Ring | 5 | 6 | `r_finger3` |
| Thumb | 7 | 8 | `r_finger4` |

Two hands **on the same bus** need different IDs; the repo uses 11-18 for the left hand
(see `assets/Both_Hands-IDs.jpg` and chapter 8).

## 1.7 The named poses

The Onshape CAD document has a "named positions" table (`assets/Named_Pos.jpg`). The demos use
these values in degrees, `(motor1, motor2)` per finger, **right hand**:

| Pose | Index 1,2 | Middle 3,4 | Ring 5,6 | Thumb 7,8 |
|---|---|---|---|---|
| Open straight | -35, 35 | -35, 35 | -35, 35 | -35, 35 |
| Open spread | 4, 90 | -32, 32 | -90, -4 | -90, -4 |
| Open clenched | -60, 0 | -35, 35 | 0, 70 | -4, 90 |
| Middle | 0, 0 | 0, 0 | 0, 0 | 0, 0 |
| Closed | 90, -90 | 90, -90 | 90, -90 | 90, -90 |
| Victory | -15, 65 | -65, 15 | 90, -90 | 90, -90 |
| Scissors | -50, 20 | -20, 50 | 90, -90 | 90, -90 |
| Index pointing | -40, 40 | 90, -90 | 90, -90 | 90, -90 |
| Nonono right | -10, 80 | 90, -90 | 90, -90 | 90, -90 |
| Nonono left | -80, 10 | 90, -90 | 90, -90 | 90, -90 |
| Perfect | 45, -45 | 0, 0 | -20, 20 | 65, 12 |
| Pinched | 90, -90 | 90, -90 | 90, -90 | -5, -75 |

## 1.8 Tour of the repository (what is software, what is not)

| Folder | What it is | Do you need it? |
|---|---|---|
| `cad/`, `docs/*.pdf`, `assets/` | 3D files, assembly/printing guides, pictures | reference only |
| `PythonExample/` | 4 Python scripts: calibrate + demo, with `rustypot` | **Path A**, chapters 6-8 |
| `ArduinoExample/` | 2 Arduino sketches with `SCServo` | **Path B**, chapter 9 |
| `Demo/AHControl/` | Rust: dora node that drives the motors + CLI tools (change ID, goto, get/set zeros) | advanced, ch 11 |
| `Demo/AHSimulation/` | Python: MuJoCo model of the hand + inverse kinematics node + an example | advanced, ch 12 |
| `Demo/HandTracking/` | Python: webcam + MediaPipe hand tracking node | advanced, ch 13 |
| `Demo/*.yml` | dora dataflows that connect the nodes | advanced, ch 13 |

## 1.9 The libraries

* **`rustypot`** (Pollen Robotics): a Rust library with Python bindings that speaks the
  Feetech/Dynamixel protocols. In Python: `from rustypot import Scs0009PyController`.
  Every register above becomes a method: `read_present_position(id)`,
  `write_goal_position(id, radians)`, `write_goal_speed(id, rad_per_s)`,
  `write_torque_enable(id, 0|1)`, `sync_write_goal_position(ids, values)`,
  `ping(id)`, `write_id(id, new_id)`… Units: **radians** and **rad/s**, 0 rad = raw 511.
  `read_*` methods return a **list** (`read_present_position(1)[0]`). Errors (e.g. no
  answer) raise `RuntimeError`.
* **`SCServo`** (Feetech, Arduino): class `SCSCL` for SCS servos, raw units.
* **dora-rs**: a framework that runs several programs ("nodes") and passes data between them (chapter 10).
* **MuJoCo + mink**: physics simulator + inverse-kinematics solver (chapter 12).
* **MediaPipe**: Google's hand-landmark detector (chapter 13).

Next → [02 Hardware setup](02_Hardware_Setup.md)
