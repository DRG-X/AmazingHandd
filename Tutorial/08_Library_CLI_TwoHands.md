# 08 · Level up: a hand library, an interactive CLI, two hands

`hand_demo.py` works, but everything is global and copy-pasted. Here we package it into a
class you can import from any future project, then build a small keyboard shell on top of it.

## 8.1 `python/amazing_hand.py`: the driver class

```python
"""
amazing_hand.py - a small reusable driver for ONE Amazing Hand.

Everything hand_demo.py does, packaged as a class you can import from any script:

    from amazing_hand import AmazingHand
    with AmazingHand("COM11", side="right", middle_pos=[3, 0, -5, -8, -2, 5, -12, 0]) as hand:
        hand.pose("victory")
        hand.finger("index", flex=45, abd=-10)
"""
import time

import numpy as np
from rustypot import Scs0009PyController

FINGERS = ("index", "middle", "ring", "thumb")

# (motor 1 ID, motor 2 ID) for each finger of a standalone hand (right OR left).
DEFAULT_IDS = {"index": (1, 2), "middle": (3, 4), "ring": (5, 6), "thumb": (7, 8)}

# Mechanical safety limit for a motor angle relative to its middle position (deg).
ANGLE_LIMIT = 90

CLOSED = (90, -90)
OPEN = (-35, 35)

# Named poses for a RIGHT hand, (motor1 deg, motor2 deg) per finger.
# Values come from the Onshape "named positions" table and the demo scripts.
POSES_RIGHT = {
    "open":     {"index": OPEN, "middle": OPEN, "ring": OPEN, "thumb": OPEN},
    "middle":   {"index": (0, 0), "middle": (0, 0), "ring": (0, 0), "thumb": (0, 0)},
    "closed":   {"index": CLOSED, "middle": CLOSED, "ring": CLOSED, "thumb": CLOSED},
    "spread":   {"index": (4, 90), "middle": (-32, 32), "ring": (-90, -4), "thumb": (-90, -4)},
    "clenched": {"index": (-60, 0), "middle": (-35, 35), "ring": (0, 70), "thumb": (-4, 90)},
    "pointing": {"index": (-40, 40), "middle": CLOSED, "ring": CLOSED, "thumb": CLOSED},
    "victory":  {"index": (-15, 65), "middle": (-65, 15), "ring": CLOSED, "thumb": CLOSED},
    "scissors": {"index": (-50, 20), "middle": (-20, 50), "ring": CLOSED, "thumb": CLOSED},
    "perfect":  {"index": (50, -50), "middle": (0, 0), "ring": (-20, 20), "thumb": (65, 12)},
    "pinched":  {"index": CLOSED, "middle": CLOSED, "ring": CLOSED, "thumb": (0, -75)},
}


def mirror(angles):
    """Right-hand angles -> left-hand angles: same flexion, opposite abduction."""
    a1, a2 = angles
    return (-a2, -a1)


def flex_abd_to_motors(flex, abd):
    """flex: + closes the finger, - opens it. abd: sideways tilt. Returns (motor1, motor2) deg."""
    return (flex + abd, abd - flex)


def motors_to_flex_abd(a1, a2):
    return ((a1 - a2) / 2, (a1 + a2) / 2)


class AmazingHand:
    def __init__(self, port, side="right", middle_pos=None, ids=None,
                 baudrate=1_000_000, timeout=0.05, speed=7):
        if side not in ("right", "left"):
            raise ValueError("side must be 'right' or 'left'")
        self.side = side
        self.ids = dict(ids or DEFAULT_IDS)
        self.middle_pos = list(middle_pos or [0] * 8)
        if len(self.middle_pos) != 8:
            raise ValueError("middle_pos needs 8 values (index1, index2, ..., thumb2)")
        self.speed = speed
        self.c = Scs0009PyController(serial_port=port, baudrate=baudrate, timeout=timeout)

    # ---------------------------------------------------------------- basics --
    @property
    def all_ids(self):
        return [i for f in FINGERS for i in self.ids[f]]

    def _offsets(self, finger):
        k = 2 * FINGERS.index(finger)
        return self.middle_pos[k], self.middle_pos[k + 1]

    def torque(self, on):
        for servo_id in self.all_ids:
            self.c.write_torque_enable(servo_id, 1 if on else 0)
            time.sleep(0.001)

    def check(self):
        """Return the list of IDs that do NOT answer."""
        missing = []
        for servo_id in self.all_ids:
            try:
                self.c.ping(servo_id)
            except RuntimeError:
                missing.append(servo_id)
        return missing

    # ------------------------------------------------------------- movement --
    def move(self, finger, a1, a2, speed=None):
        """Move one finger. a1/a2 = motor angles in degrees relative to the calibrated middle."""
        if finger not in FINGERS:
            raise ValueError(f"unknown finger {finger!r}, use one of {FINGERS}")
        speed = self.speed if speed is None else speed
        a1 = float(np.clip(a1, -ANGLE_LIMIT, ANGLE_LIMIT))
        a2 = float(np.clip(a2, -ANGLE_LIMIT, ANGLE_LIMIT))
        (id1, id2), (m1, m2) = self.ids[finger], self._offsets(finger)
        self.c.write_goal_speed(id1, speed)
        self.c.write_goal_speed(id2, speed)
        self.c.write_goal_position(id1, float(np.deg2rad(m1 + a1)))
        self.c.write_goal_position(id2, float(np.deg2rad(m2 + a2)))
        time.sleep(0.002)

    def finger(self, finger, flex, abd=0.0, speed=None):
        """Move one finger using intuitive flexion / abduction angles (deg)."""
        if self.side == "left":
            abd = -abd
        self.move(finger, *flex_abd_to_motors(flex, abd), speed=speed)

    def pose(self, name, speed=None):
        """Apply a named pose from POSES_RIGHT (mirrored automatically for a left hand)."""
        if name not in POSES_RIGHT:
            raise ValueError(f"unknown pose {name!r}, use one of {sorted(POSES_RIGHT)}")
        for f in FINGERS:
            angles = POSES_RIGHT[name][f]
            if self.side == "left":
                angles = mirror(angles)
            self.move(f, *angles, speed=speed)

    # -------------------------------------------------------------- sensing --
    def read_angles(self):
        """Present motor angles in degrees, relative to the calibrated middle."""
        out = {}
        for f in FINGERS:
            (id1, id2), (m1, m2) = self.ids[f], self._offsets(f)
            p1 = np.rad2deg(self.c.read_present_position(id1)[0]) - m1
            p2 = np.rad2deg(self.c.read_present_position(id2)[0]) - m2
            out[f] = (round(float(p1), 1), round(float(p2), 1))
        return out

    def read_status(self):
        """Temperature (C), voltage (V) and load (raw, signed) of every servo."""
        status = {}
        for servo_id in self.all_ids:
            status[servo_id] = {
                "temp": self.c.read_present_temperature(servo_id)[0],
                "volt": self.c.read_present_voltage(servo_id)[0] / 10,
                "load": self.c.read_present_load(servo_id)[0],
            }
        return status

    # ------------------------------------------------------------ lifecycle --
    def close(self):
        """Open the hand gently, then release torque."""
        try:
            self.pose("open", speed=3)
            time.sleep(1.0)
        finally:
            self.torque(False)

    def __enter__(self):
        self.torque(True)
        return self

    def __exit__(self, *exc):
        self.close()
        return False
```

Design notes:

* **Data instead of code:** the gestures are a dictionary (`POSES_RIGHT`) instead of 12 functions.
  Add your own poses by adding a line.
* **`mirror()`** implements the left-hand rule from chapter 1.4, so the dictionary is written once.
* **`flex_abd_to_motors()`** lets you think in "close 45°, tilt −10°" instead of motor angles:
  `hand.finger("index", flex=45, abd=-10)`. For a right hand, positive `abd` tilts the finger
  the way the index goes in the *spread* pose.
* **Safety clamp:** every motor angle is clipped to ±90° around the calibrated middle.
* **Context manager** (`with AmazingHand(...) as hand:`) turns torque on at the start and
  **always** opens the hand + turns torque off at the end, even after an exception or Ctrl+C.
* **Sensing:** `read_angles()` (calibrated, in degrees) and `read_status()` (temperature,
  voltage, load). Load is the first step towards "smart grasping" (stop closing when load rises).
* `float(...)` around numpy values: rustypot wants plain Python floats.

## 8.2 `python/hand_cli.py`: drive the hand from the keyboard

```python
"""
hand_cli.py - drive the hand from the keyboard, built on amazing_hand.py.

Commands:
  <pose>                      e.g. open, closed, victory, perfect ...
  f <finger> <flex> [abd]     e.g. f index 45 -10
  m <finger> <a1> <a2>        raw motor angles, e.g. m thumb 65 12
  read                        present angles
  status                      temperature / voltage / load
  speed <rad/s>               e.g. speed 3
  quit
"""
from amazing_hand import FINGERS, POSES_RIGHT, AmazingHand

SERIAL_PORT = "COM11"
SIDE = "right"
MIDDLE_POS = [3, 0, -5, -8, -2, 5, -12, 0]   # <-- your calibration


def main():
    with AmazingHand(SERIAL_PORT, side=SIDE, middle_pos=MIDDLE_POS) as hand:
        missing = hand.check()
        if missing:
            print(f"WARNING: no answer from IDs {missing}")
        print(__doc__)
        print("Poses:", ", ".join(sorted(POSES_RIGHT)))
        while True:
            try:
                words = input("hand> ").strip().lower().split()
            except (EOFError, KeyboardInterrupt):
                break
            if not words:
                continue
            cmd, args = words[0], words[1:]
            try:
                if cmd in ("quit", "exit", "q"):
                    break
                elif cmd in POSES_RIGHT:
                    hand.pose(cmd)
                elif cmd == "f" and len(args) >= 2 and args[0] in FINGERS:
                    hand.finger(args[0], float(args[1]), float(args[2]) if len(args) > 2 else 0.0)
                elif cmd == "m" and len(args) == 3 and args[0] in FINGERS:
                    hand.move(args[0], float(args[1]), float(args[2]))
                elif cmd == "read":
                    for finger, angles in hand.read_angles().items():
                        print(f"  {finger:7s} {angles}")
                elif cmd == "status":
                    for servo_id, s in hand.read_status().items():
                        print(f"  ID {servo_id}: {s['temp']} C  {s['volt']:.1f} V  load {s['load']}")
                elif cmd == "speed" and len(args) == 1:
                    hand.speed = float(args[0])
                else:
                    print("?? see the command list above")
            except (ValueError, RuntimeError) as e:
                print("error:", e)
    print("Hand opened, torque off. Bye.")


if __name__ == "__main__":
    main()
```

Run it and play:

```
hand> victory
hand> f index 60 0
hand> f index 0 20
hand> m thumb 65 12
hand> read
hand> status
hand> speed 2
hand> closed
hand> quit
```

✅ **Checkpoint:** each command moves the hand and `status` shows temperatures below ~45 °C
after a few minutes of play.

This is also the best tool for **designing new gestures**: try values with `m finger a1 a2`,
then add the winners to `POSES_RIGHT`.

## 8.3 Two hands on one bus: `python/hand_demo_both.py` (optional)

Repo: `PythonExample/AmazingHand_Demo_Both.py`. Only needed if you build a left hand too and
put both on **one** serial bus (e.g. on a robot). Then the left hand needs other IDs; the
repo uses **11-18** (`assets/Both_Hands-IDs.jpg`). Set them with `change_id.py` (chapter 5).

The repo file repeats `if Hand == 1 … if Hand == 2 …` in every `Move_X` function (~330 lines).
The version below does exactly the same moves with one ID table and one `Move()` function:

```python
"""
hand_demo_both.py  (repo: PythonExample/AmazingHand_Demo_Both.py, compacted)

Same gestures on a right hand (IDs 1-8) and a left hand (IDs 11-18) sharing ONE bus.
The original file has 4 copies of Move_X with an `if Hand == ...` inside each;
here one table + one Move() function do the same job.
"""
import time

import numpy as np
from rustypot import Scs0009PyController

SERIAL_PORT = "COM5"

MaxSpeed = 7
CloseSpeed = 3

RIGHT, LEFT = 1, 2

# (motor1 ID, motor2 ID) of every finger, per hand. Same values as the original script.
# !! Check with read_positions.py which finger 11/12 really moves on YOUR left hand
# !! (the IDs picture in the README puts 15/16 next to the thumb).
IDS = {
    RIGHT: {"index": (1, 2), "middle": (3, 4), "ring": (5, 6), "thumb": (7, 8)},
    LEFT: {"index": (11, 12), "middle": (13, 14), "ring": (15, 16), "thumb": (17, 18)},
}
# Calibration results in degrees, in the order index1, index2, middle1, middle2, ring1, ring2, thumb1, thumb2
MIDDLE_POS = {
    RIGHT: [3, 0, -8, -13, 2, -5, -12, -5],
    LEFT: [3, -3, -1, -10, 5, 2, -7, 3],
}
FINGER_INDEX = {"index": 0, "middle": 2, "ring": 4, "thumb": 6}

c = Scs0009PyController(serial_port=SERIAL_PORT, baudrate=1000000, timeout=0.05)


def Move(finger, Angle_1, Angle_2, Speed, Hand):
    id_1, id_2 = IDS[Hand][finger]
    k = FINGER_INDEX[finger]
    c.write_goal_speed(id_1, Speed)
    time.sleep(0.0002)
    c.write_goal_speed(id_2, Speed)
    time.sleep(0.0002)
    c.write_goal_position(id_1, np.deg2rad(MIDDLE_POS[Hand][k] + Angle_1))
    c.write_goal_position(id_2, np.deg2rad(MIDDLE_POS[Hand][k + 1] + Angle_2))
    time.sleep(0.0002)


def Both(finger, right_angles, left_angles, Speed=MaxSpeed):
    """Move the same finger on both hands. *_angles = (Angle_1, Angle_2)."""
    Move(finger, *right_angles, Speed, RIGHT)
    Move(finger, *left_angles, Speed, LEFT)


def OpenHand():
    for f in ("index", "middle", "ring", "thumb"):
        Both(f, (-35, 35), (-35, 35))


def CloseHand():
    for f in ("index", "middle", "ring"):
        Both(f, (90, -90), (90, -90), CloseSpeed)
    Both("thumb", (90, -90), (90, -90), CloseSpeed + 4)  # faster: thumb must pass under the index


def OpenHand_Progressive():
    for f in ("index", "middle", "ring"):
        Both(f, (-35, 35), (-35, 35), MaxSpeed - 2)
        time.sleep(0.2)
    Both("thumb", (-35, 35), (-35, 35), MaxSpeed - 2)


def SpreadHand():
    Both("index", (4, 90), (-90, 0))
    Both("middle", (-32, 32), (-32, 32))
    Both("ring", (-90, -4), (-4, 90))
    Both("thumb", (-90, -4), (-4, 90))


def ClenchHand():
    Both("index", (-60, 0), (0, 60))
    Both("middle", (-35, 35), (-35, 35))
    Both("ring", (0, 70), (-70, 0))
    Both("thumb", (-4, 90), (-90, -4))


def Index_Pointing():
    Both("index", (-40, 40), (-40, 40))
    for f in ("middle", "ring", "thumb"):
        Both(f, (90, -90), (90, -90))


def Nonono():
    Index_Pointing()
    for i in range(3):
        time.sleep(0.2)
        Both("index", (-10, 80), (-10, 80))
        time.sleep(0.2)
        Both("index", (-80, 10), (-80, 10))
    Both("index", (-35, 35), (-35, 35))
    time.sleep(0.4)


def Perfect():
    Both("index", (55, -55), (55, -55), MaxSpeed - 3)
    Both("middle", (0, 0), (0, 0))
    Both("ring", (-20, 20), (-20, 20))
    Both("thumb", (85, 10), (-10, -85))


def Victory():
    Both("index", (-15, 65), (-65, 15))
    Both("middle", (-65, 15), (-15, 65))
    Both("ring", (90, -90), (90, -90))
    Both("thumb", (90, -90), (90, -90))


def Pinched():
    for f in ("index", "middle", "ring"):
        Both(f, (90, -90), (90, -90))
    Both("thumb", (5, -75), (75, -5))


def Scissors():
    Victory()
    for i in range(3):
        time.sleep(0.2)
        Both("index", (-50, 20), (-20, 50))
        Both("middle", (-20, 50), (-50, 20))
        time.sleep(0.2)
        Both("index", (-15, 65), (-65, 15))
        Both("middle", (-65, 15), (-15, 65))


def MiddleFinger():  # "Fuck()" in the original
    Both("index", (90, -90), (90, -90))
    Both("middle", (-35, 35), (-35, 35))
    Both("ring", (90, -90), (90, -90))
    Both("thumb", (5, -75), (75, -5))


ALL_IDS = [i for hand in IDS.values() for pair in hand.values() for i in pair]


def main():
    for servo_id in ALL_IDS:
        c.write_torque_enable(servo_id, 1)
    try:
        while True:
            OpenHand(); time.sleep(0.5)
            CloseHand(); time.sleep(4)
            OpenHand_Progressive(); time.sleep(0.5)
            SpreadHand(); time.sleep(0.6)
            ClenchHand(); time.sleep(0.6)
            OpenHand(); time.sleep(0.2)
            Index_Pointing(); time.sleep(0.4)
            Nonono(); time.sleep(0.5)
            OpenHand(); time.sleep(0.3)
            Perfect(); time.sleep(0.8)
            OpenHand(); time.sleep(0.4)
            Victory(); time.sleep(0.5)
            Scissors(); time.sleep(0.5)
            OpenHand(); time.sleep(0.4)
            Pinched(); time.sleep(1.5)
            MiddleFinger(); time.sleep(0.8)
    except KeyboardInterrupt:
        OpenHand()
        time.sleep(1)
        for servo_id in ALL_IDS:
            c.write_torque_enable(servo_id, 0)
        print("\nTorque off, bye.")


if __name__ == "__main__":
    main()
```

> ⚠️ **Inconsistency in the repo, check yours:** `AmazingHand_Demo_Both.py` treats IDs 11/12
> as the left **index**, but the IDs picture (`assets/Both_Hands-IDs.jpg`) and
> `Demo/AHControl/config/2hands.toml` put **15/16** next to the thumb, which is the index.
> Use `read_positions.py` (with IDs 11-18) to see which finger 11/12 moves on your hand, and
> fix the `IDS[LEFT]` table accordingly. One table edit fixes every gesture.

Next → [09 Arduino path](09_Arduino.md) (optional) or [10 Advanced demo](10_Advanced_Setup.md)
