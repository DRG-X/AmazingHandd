# 07 · The full hand demo

Repo: `PythonExample/AmazingHand_Demo.py`. With the 8 IDs set and calibrated, this plays
all the gestures in a loop (the video in the README).

## 7.1 `python/hand_demo.py`

```python
"""
hand_demo.py  (repo: PythonExample/AmazingHand_Demo.py)

Loops through a sequence of gestures on one hand (IDs 1..8).
"""
import time

import numpy as np
from rustypot import Scs0009PyController

# ---------------------------------------------------------------- settings --
SERIAL_PORT = "COM11"

Side = 1  # 1 => Right hand, 2 => Left hand

# Speeds in rad/s
MaxSpeed = 7
CloseSpeed = 3

# Your calibration results, in degrees:
#            [ID1, ID2, ID3, ID4, ID5, ID6, ID7, ID8]
MiddlePos = [3, 0, -5, -8, -2, 5, -12, 0]   # <-- replace with YOUR values

c = Scs0009PyController(
    serial_port=SERIAL_PORT,
    baudrate=1000000,
    timeout=0.5,
)


# ------------------------------------------------------------------- main --
def main():
    for servo_id in range(1, 9):          # original only enabled ID 1
        c.write_torque_enable(servo_id, 1)

    try:
        while True:
            OpenHand()
            time.sleep(0.5)

            CloseHand()
            time.sleep(3)

            OpenHand_Progressive()
            time.sleep(0.5)

            SpreadHand()
            time.sleep(0.6)
            ClenchHand()
            time.sleep(0.6)

            OpenHand()
            time.sleep(0.2)

            Index_Pointing()
            time.sleep(0.4)
            Nonono()
            time.sleep(0.5)

            OpenHand()
            time.sleep(0.3)

            Perfect()
            time.sleep(0.8)

            OpenHand()
            time.sleep(0.4)

            Victory()
            time.sleep(1)
            Scissors()
            time.sleep(0.5)

            OpenHand()
            time.sleep(0.4)

            Pinched()
            time.sleep(1)

            MiddleFinger()
            time.sleep(0.8)
    except KeyboardInterrupt:
        OpenHand()
        time.sleep(1)
        for servo_id in range(1, 9):
            c.write_torque_enable(servo_id, 0)
        print("\nHand opened, torque off. Bye.")


# --------------------------------------------------------------- gestures --
# Each Move_X(Angle_1, Angle_2, Speed): Angle_1 for motor 1, Angle_2 for motor 2,
# in degrees, relative to the calibrated middle position.
#   (+a, -a)  -> finger closes (flexion)       e.g. ( 90, -90) = closed
#   (-a, +a)  -> finger opens  (extension)     e.g. (-35,  35) = open straight
#   (+b, +b)  -> finger tilts sideways (abduction/adduction)

def OpenHand():
    Move_Index(-35, 35, MaxSpeed)
    Move_Middle(-35, 35, MaxSpeed)
    Move_Ring(-35, 35, MaxSpeed)
    Move_Thumb(-35, 35, MaxSpeed)


def CloseHand():
    Move_Index(90, -90, CloseSpeed)
    Move_Middle(90, -90, CloseSpeed)
    Move_Ring(90, -90, CloseSpeed)
    Move_Thumb(90, -90, CloseSpeed + 1)   # thumb slightly faster so it passes under the index


def OpenHand_Progressive():
    Move_Index(-35, 35, MaxSpeed - 2)
    time.sleep(0.2)
    Move_Middle(-35, 35, MaxSpeed - 2)
    time.sleep(0.2)
    Move_Ring(-35, 35, MaxSpeed - 2)
    time.sleep(0.2)
    Move_Thumb(-35, 35, MaxSpeed - 2)


def SpreadHand():
    if Side == 1:  # Right hand
        Move_Index(4, 90, MaxSpeed)
        Move_Middle(-32, 32, MaxSpeed)
        Move_Ring(-90, -4, MaxSpeed)
        Move_Thumb(-90, -4, MaxSpeed)
    if Side == 2:  # Left hand. The original had (-60, 0) for the index (copy/paste slip);
        #            (-90, 0) is the value used in AmazingHand_Demo_Both.py.
        Move_Index(-90, 0, MaxSpeed)
        Move_Middle(-35, 35, MaxSpeed)
        Move_Ring(-4, 90, MaxSpeed)
        Move_Thumb(-4, 90, MaxSpeed)


def ClenchHand():
    if Side == 1:
        Move_Index(-60, 0, MaxSpeed)
        Move_Middle(-35, 35, MaxSpeed)
        Move_Ring(0, 70, MaxSpeed)
        Move_Thumb(-4, 90, MaxSpeed)
    if Side == 2:
        Move_Index(0, 60, MaxSpeed)
        Move_Middle(-35, 35, MaxSpeed)
        Move_Ring(-70, 0, MaxSpeed)
        Move_Thumb(-90, -4, MaxSpeed)


def Index_Pointing():
    Move_Index(-40, 40, MaxSpeed)
    Move_Middle(90, -90, MaxSpeed)
    Move_Ring(90, -90, MaxSpeed)
    Move_Thumb(90, -90, MaxSpeed)


def Nonono():
    Index_Pointing()
    for i in range(3):
        time.sleep(0.2)
        Move_Index(-10, 80, MaxSpeed)
        time.sleep(0.2)
        Move_Index(-80, 10, MaxSpeed)

    Move_Index(-35, 35, MaxSpeed)
    time.sleep(0.4)


def Perfect():
    if Side == 1:
        Move_Index(50, -50, MaxSpeed)
        Move_Middle(0, -0, MaxSpeed)
        Move_Ring(-20, 20, MaxSpeed)
        Move_Thumb(65, 12, MaxSpeed)
    if Side == 2:
        Move_Index(50, -50, MaxSpeed)
        Move_Middle(0, -0, MaxSpeed)
        Move_Ring(-20, 20, MaxSpeed)
        Move_Thumb(-12, -65, MaxSpeed)


def Victory():
    if Side == 1:
        Move_Index(-15, 65, MaxSpeed)
        Move_Middle(-65, 15, MaxSpeed)
        Move_Ring(90, -90, MaxSpeed)
        Move_Thumb(90, -90, MaxSpeed)
    if Side == 2:
        Move_Index(-65, 15, MaxSpeed)
        Move_Middle(-15, 65, MaxSpeed)
        Move_Ring(90, -90, MaxSpeed)
        Move_Thumb(90, -90, MaxSpeed)


def Pinched():
    if Side == 1:
        Move_Index(90, -90, MaxSpeed)
        Move_Middle(90, -90, MaxSpeed)
        Move_Ring(90, -90, MaxSpeed)
        Move_Thumb(0, -75, MaxSpeed)
    if Side == 2:
        Move_Index(90, -90, MaxSpeed)
        Move_Middle(90, -90, MaxSpeed)
        Move_Ring(90, -90, MaxSpeed)
        Move_Thumb(75, 5, MaxSpeed)


def Scissors():
    Victory()
    if Side == 1:
        for i in range(3):
            time.sleep(0.2)
            Move_Index(-50, 20, MaxSpeed)
            Move_Middle(-20, 50, MaxSpeed)

            time.sleep(0.2)
            Move_Index(-15, 65, MaxSpeed)
            Move_Middle(-65, 15, MaxSpeed)
    if Side == 2:
        for i in range(3):
            time.sleep(0.2)
            Move_Index(-20, 50, MaxSpeed)
            Move_Middle(-50, 20, MaxSpeed)

            time.sleep(0.2)
            Move_Index(-65, 15, MaxSpeed)
            Move_Middle(-15, 65, MaxSpeed)


def MiddleFinger():  # called "Fuck()" in the original file
    if Side == 1:
        Move_Index(90, -90, MaxSpeed)
        Move_Middle(-35, 35, MaxSpeed)
        Move_Ring(90, -90, MaxSpeed)
        Move_Thumb(0, -75, MaxSpeed)
    if Side == 2:
        Move_Index(90, -90, MaxSpeed)
        Move_Middle(-35, 35, MaxSpeed)
        Move_Ring(90, -90, MaxSpeed)
        Move_Thumb(75, 0, MaxSpeed)


# ---------------------------------------------------------- finger moves --
def Move_Index(Angle_1, Angle_2, Speed):
    c.write_goal_speed(1, Speed)
    time.sleep(0.0002)
    c.write_goal_speed(2, Speed)
    time.sleep(0.0002)
    Pos_1 = np.deg2rad(MiddlePos[0] + Angle_1)
    Pos_2 = np.deg2rad(MiddlePos[1] + Angle_2)
    c.write_goal_position(1, Pos_1)
    c.write_goal_position(2, Pos_2)
    time.sleep(0.005)


def Move_Middle(Angle_1, Angle_2, Speed):
    c.write_goal_speed(3, Speed)
    time.sleep(0.0002)
    c.write_goal_speed(4, Speed)
    time.sleep(0.0002)
    Pos_1 = np.deg2rad(MiddlePos[2] + Angle_1)
    Pos_2 = np.deg2rad(MiddlePos[3] + Angle_2)
    c.write_goal_position(3, Pos_1)
    c.write_goal_position(4, Pos_2)
    time.sleep(0.005)


def Move_Ring(Angle_1, Angle_2, Speed):
    c.write_goal_speed(5, Speed)
    time.sleep(0.0002)
    c.write_goal_speed(6, Speed)
    time.sleep(0.0002)
    Pos_1 = np.deg2rad(MiddlePos[4] + Angle_1)
    Pos_2 = np.deg2rad(MiddlePos[5] + Angle_2)
    c.write_goal_position(5, Pos_1)
    c.write_goal_position(6, Pos_2)
    time.sleep(0.005)


def Move_Thumb(Angle_1, Angle_2, Speed):
    c.write_goal_speed(7, Speed)
    time.sleep(0.0002)
    c.write_goal_speed(8, Speed)
    time.sleep(0.0002)
    Pos_1 = np.deg2rad(MiddlePos[6] + Angle_1)
    Pos_2 = np.deg2rad(MiddlePos[7] + Angle_2)
    c.write_goal_position(7, Pos_1)
    c.write_goal_position(8, Pos_2)
    time.sleep(0.005)


if __name__ == "__main__":
    main()
```

## 7.2 How it is built

**Three layers:**

1. **Settings**: port, `Side`, speeds (rad/s), your `MiddlePos` list.
2. **Finger layer**: `Move_Index / Move_Middle / Move_Ring / Move_Thumb(Angle_1, Angle_2, Speed)`.
   Each one:
   * sets the speed of its two servos,
   * adds the calibration offset: `MiddlePos[k] + Angle` (degrees),
   * converts to radians and writes the two goal positions.

   The servo IDs are hard-coded: index = 1,2 (`MiddlePos[0]`, `[1]`), middle = 3,4 (`[2]`,`[3]`),
   ring = 5,6, thumb = 7,8. The tiny `time.sleep(0.0002)` / `0.005` give the bus some breathing room.
3. **Gesture layer**: `OpenHand()`, `Victory()`… Each is just four `Move_X` calls with the values
   from the named-poses table (chapter 1.7). Animated gestures (`Nonono`, `Scissors`) alternate
   two poses with short sleeps.

**Reading a gesture:** `Move_Index(-15, 65, …)`: flex = (−15−65)/2 = −40 (open),
abd = (−15+65)/2 = +25 (tilted), i.e. the index is spread for the "V".

**Left hand (`Side = 2`):** the same IDs 1-8 at the same places, but asymmetric gestures use
the mirrored values `(-motor2, -motor1)`. Symmetric ones (open, close, pointing) don't need a branch.

**Speeds:** `MaxSpeed = 7` rad/s (~400°/s). `CloseSpeed = 3` for a softer fist. In `CloseHand`
the thumb gets `+1` so it arrives first and folds under the index.

**`main()`:** torque on for the 8 servos, then the choreography: gesture → `time.sleep(hold)` → next.

## 7.3 Differences from the repo file

| Change | Why |
|---|---|
| Torque enabled on IDs 1-8 (repo: only ID 1) | explicit is better. It worked before only because writing a goal position switches torque on |
| `try/except KeyboardInterrupt`: open hand, torque off | safe exit with Ctrl+C |
| `Fuck()` renamed `MiddleFinger()` | same gesture |
| Left `SpreadHand` index `(-60, 0)` → `(-90, 0)` | the repo's single-hand file has a copy/paste slip. `(-90, 0)` is the value from `AmazingHand_Demo_Both.py` and matches the mirror rule |
| Removed unused `t = time.time() - t0` and commented trial code | clutter |

## 7.4 Run it

1. Put your `MiddlePos` values in and set `SERIAL_PORT`.
2. First run: set `MaxSpeed = 3` and `CloseSpeed = 2`, hold the hand (or put it on its base).
3. `python python/hand_demo.py`, then Ctrl+C to stop.

✅ **Checkpoint:** the hand loops open → fist → progressive opening → spread → clench → pointing
→ "no no no" → perfect → victory → scissors → pinch → middle finger. Fingers never buzz at the end
of a move. If one does, its calibration is off (chapter 6) or it hits another finger (lower the angle).

Next → [08 Library, CLI, two hands](08_Library_CLI_TwoHands.md)
