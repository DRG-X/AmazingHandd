# 06 · Calibration: the middle positions

Goal: find the 8 numbers `MiddlePos = [ID1, ID2, …, ID8]` (degrees) so that every finger
closes and opens symmetrically. This is the most important step for a good-looking hand.

The assembly guide does this **finger by finger during assembly** (steps 3.x). If your hand
is already fully assembled you can still do the fine-tuning part (6.3) through the openings;
only if a horn is badly off (> ~20°) do you need to unscrew it and re-seat it one spline tooth over.

## 6.1 `python/finger_middle_pos.py`: servos to the middle

Repo: `PythonExample/AmazingHand_Hand_FingerMiddlePos.py`. Moves the two servos of one finger to 0°
(raw 511) and **holds** them there, so you can mount the horns in the "middle position".

```python
"""
finger_middle_pos.py  (repo: PythonExample/AmazingHand_Hand_FingerMiddlePos.py)

Holds the two servos of ONE finger at their middle position (0 deg) so you can
mount / check the servo horns.
"""
import time

import numpy as np
from rustypot import Scs0009PyController

SERIAL_PORT = "COM11"

ID_1 = 1          # motor 1 of the finger you are working on
ID_2 = 2          # motor 2 of the same finger
MiddlePos_1 = 0   # calibration offset of ID_1, in degrees (0 until you calibrate)
MiddlePos_2 = 0   # calibration offset of ID_2, in degrees

c = Scs0009PyController(
    serial_port=SERIAL_PORT,
    baudrate=1000000,
    timeout=0.5,
)


def main():
    c.write_torque_enable(ID_1, 1)   # 1 = torque on (servo holds its position)
    c.write_torque_enable(ID_2, 1)

    try:
        while True:
            ServosInMiddle()
            time.sleep(3)
    except KeyboardInterrupt:        # Ctrl+C
        c.write_torque_enable(ID_1, 0)   # 0 = torque off (servo goes limp)
        c.write_torque_enable(ID_2, 0)
        print("\nTorque off, bye.")


def ServosInMiddle():
    c.write_goal_speed(ID_1, 6)      # speed in rad/s (6 rad/s ~ 340 deg/s)
    c.write_goal_speed(ID_2, 6)
    Pos_1 = np.deg2rad(MiddlePos_1)  # rustypot wants radians, 0 rad = middle (raw 511)
    Pos_2 = np.deg2rad(MiddlePos_2)
    c.write_goal_position(ID_1, Pos_1)
    c.write_goal_position(ID_2, Pos_2)
    time.sleep(0.01)


if __name__ == "__main__":
    main()
```

Line by line:

* `ID_1, ID_2`: the two servos of the finger you are working on.
* `MiddlePos_1/2 = 0`: no correction yet.
* `write_torque_enable(id, 1)`: **1 = torque on**. (The original comment says `1 = On / 2 = Off / 3 = Free`.
  That comment is misleading: register 40 uses **0 = off, 1 = on**, like the repo's Rust code.)
* The loop re-sends the goal every 3 s (harmless, and it survives a servo brown-out).
* `write_goal_speed(id, 6)`: 6 **rad/s** (≈ 340°/s).
* `np.deg2rad(...)`: rustypot positions are radians, 0 rad = middle.
* **Added:** `except KeyboardInterrupt` turns torque off on Ctrl+C.

**Use it (only if you are (re)mounting horns):** run it, then place both horns as in the
assembly guide p.22 (horn arm pointing along the servo's middle plane, as close as the spline
allows), and screw them with the M2x4 screws.

## 6.2 `python/finger_test.py`: open / close one finger

Repo: `PythonExample/AmazingHand_FingerTest.py`.

```python
"""
finger_test.py  (repo: PythonExample/AmazingHand_FingerTest.py)

Opens and closes ONE finger forever. Used to fine-tune MiddlePos_1 / MiddlePos_2.
"""
import time

import numpy as np
from rustypot import Scs0009PyController

SERIAL_PORT = "COM11"

ID_1 = 1          # change to the finger you are calibrating: 1&2, 3&4, 5&6, 7&8
ID_2 = 2
MiddlePos_1 = 0   # degrees - tune me
MiddlePos_2 = 0   # degrees - tune me

c = Scs0009PyController(
    serial_port=SERIAL_PORT,
    baudrate=1000000,
    timeout=0.5,
)


def main():
    # The original script only enabled servo 1 here; we enable both explicitly.
    c.write_torque_enable(ID_1, 1)
    c.write_torque_enable(ID_2, 1)

    try:
        while True:
            CloseFinger()
            time.sleep(3)    # <- look at the servo horns now

            OpenFinger()
            time.sleep(1)
    except KeyboardInterrupt:
        c.write_torque_enable(ID_1, 0)
        c.write_torque_enable(ID_2, 0)
        print("\nTorque off, bye.")


def CloseFinger():
    c.write_goal_speed(ID_1, 6)
    c.write_goal_speed(ID_2, 6)
    Pos_1 = np.deg2rad(MiddlePos_1 + 90)   # motor 1 turns +90 deg
    Pos_2 = np.deg2rad(MiddlePos_2 - 90)   # motor 2 turns -90 deg (it is mounted mirrored)
    c.write_goal_position(ID_1, Pos_1)
    c.write_goal_position(ID_2, Pos_2)
    time.sleep(0.01)


def OpenFinger():
    c.write_goal_speed(ID_1, 6)
    c.write_goal_speed(ID_2, 6)
    Pos_1 = np.deg2rad(MiddlePos_1 - 30)
    Pos_2 = np.deg2rad(MiddlePos_2 + 30)
    c.write_goal_position(ID_1, Pos_1)
    c.write_goal_position(ID_2, Pos_2)
    time.sleep(0.01)


if __name__ == "__main__":
    main()
```

* `CloseFinger()`: motor 1 → middle **+90°**, motor 2 → middle **−90°** (mirrored mounting, see 1.4).
* `OpenFinger()`: −30° / +30°, finger straight and slightly open.
* **Fixed:** the original only called `write_torque_enable(1, 1)` (servo **1**, whatever finger you
  test). It still works because Feetech servos switch torque on when they receive a goal
  position, but we enable both servos of the finger explicitly.

## 6.3 The fine-tuning procedure (do it for each finger)

For finger *(ID_1, ID_2)* = (1,2), then (3,4), (5,6), (7,8):

1. Set `ID_1`, `ID_2` in `finger_test.py`, `MiddlePos_1 = MiddlePos_2 = 0`.
2. Run it. The finger closes for 3 s, opens for 1 s, and repeats.
3. **While it is closed**, look at each servo horn: when the finger is closed the horn
   must be **aligned with the middle plane of the servo** (guide p.23 has the picture).
4. If a horn has not turned far enough, *increase* that servo's `MiddlePos` by a few
   degrees (e.g. `MiddlePos_1 = 3`); if it went too far, decrease it. Ctrl+C, edit, run again.
   Because the two servos are mirrored, the correct direction is not obvious: try +3, look,
   try −3, look.
5. Also check the open position: the finger should be straight, not tilted sideways.
   Remember `abd = (motor1 + motor2) / 2`: adding the **same** number to both values (e.g. both −2)
   only changes the sideways tilt, while moving them in **opposite** directions (+2 / −2) only
   changes how far the finger closes. Use this to separate the two corrections.
6. **Write the two values down.**

Typical values are between −15° and +15°. The repo author's hands: `[3, 0, -5, -8, -2, 5, -12, 0]`.

## 6.4 `python/read_positions.py`: see what the servos see

Not in the repo, but very useful: torque off, live angles on one line.

```python
"""
read_positions.py - torque OFF, then print live angles of all servos.

Move the fingers by hand and watch the numbers. Handy to:
  * check that every ID moves the finger you think it does,
  * check directions (closing a finger: motor 1 goes +, motor 2 goes -),
  * measure calibration offsets.
"""
import time

import numpy as np
from rustypot import Scs0009PyController

SERIAL_PORT = "COM11"
IDS = [1, 2, 3, 4, 5, 6, 7, 8]

c = Scs0009PyController(serial_port=SERIAL_PORT, baudrate=1_000_000, timeout=0.05)


def main():
    for servo_id in IDS:
        c.write_torque_enable(servo_id, 0)   # limp, so you can move the fingers by hand
    print("Torque is OFF. Move the fingers by hand. Ctrl+C to quit.\n")

    try:
        while True:
            cells = []
            for servo_id in IDS:
                try:
                    deg = np.rad2deg(c.read_present_position(servo_id)[0])
                    cells.append(f"{servo_id}:{deg:+6.1f}")
                except RuntimeError:
                    cells.append(f"{servo_id}:  ----")
            print("  ".join(cells), end="\r", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nBye.")


if __name__ == "__main__":
    main()
```

Uses:

* **ID check** (chapter 5.4): bend each finger; exactly two numbers change.
* **Direction check:** closing a finger → motor 1 **+**, motor 2 **−**.
* **Alternative calibration:** gently hold a finger in the "Middle" pose (see `assets/Named_Pos.jpg`, all motors 0°):
  the numbers you read are roughly the `MiddlePos` values. The repo's Rust tool
  `get_zeros` (chapter 11) is built on the same idea.

## 6.5 Your calibration table

At the end you have something like:

```python
#            ID1 ID2 ID3 ID4 ID5 ID6 ID7 ID8
MiddlePos = [3,  0,  -5, -8, -2,  5, -12, 0]
```

Keep it. It goes into `hand_demo.py` (ch 7), `hand_cli.py` (ch 8), and converted to radians
into the Rust TOML config (ch 11). For Arduino it is `511 + deg/0.293`.

✅ **Checkpoint:** each finger closes into a clean fist position and opens straight,
without buzzing at either end.

Next → [07 The full hand demo](07_Hand_Demo.md)
