# 05 · First contact: scan the bus and set the IDs

## 5.1 `python/scan_bus.py`: who is on the bus?

This is the first script to run, and the first thing to run again whenever something
doesn't work.

```python
"""
scan_bus.py - find every servo on the bus and print its state.
Run this first, every time something does not work.
"""
import time

import numpy as np
from rustypot import Scs0009PyController

SERIAL_PORT = "COM11"       # Windows: "COMx" | Linux: "/dev/ttyACM0" or "/dev/ttyUSB0" | macOS: "/dev/tty.usbmodemXXXX"
BAUDRATE = 1_000_000        # SCS0009 factory default
IDS_TO_SCAN = range(1, 21)  # 1..20 covers a right hand (1-8) and a left hand (11-18)


def main():
    c = Scs0009PyController(serial_port=SERIAL_PORT, baudrate=BAUDRATE, timeout=0.05)

    found = []
    for servo_id in IDS_TO_SCAN:
        try:
            alive = c.ping(servo_id)
        except RuntimeError:        # rustypot raises on timeout (nobody answered)
            alive = False
        if not alive:
            continue

        found.append(servo_id)
        pos_deg = np.rad2deg(c.read_present_position(servo_id)[0])  # read_* returns a list
        volts = c.read_present_voltage(servo_id)[0] / 10               # unit is 0.1 V
        temp = c.read_present_temperature(servo_id)[0]                 # deg C
        print(f"ID {servo_id:3d} | position {pos_deg:+7.1f} deg | {volts:4.1f} V | {temp:3d} C")
        time.sleep(0.005)

    print(f"\nFound {len(found)} servo(s): {found}")
    if not found:
        print("Nothing answered: check power, port name, baudrate and the data cable.")


if __name__ == "__main__":
    main()
```

What's going on:

* `Scs0009PyController(serial_port, baudrate, timeout)` opens the port. `timeout=0.05` s
  is plenty at 1 Mbaud and keeps the scan of absent IDs fast.
* `c.ping(id)`: an absent servo means no answer, and `rustypot` raises `RuntimeError`, which we catch.
* `read_*` returns a **list** with one value, hence the `[0]`.
* Voltage register is in 0.1 V, so `52` → `5.2 V`.

Run it with the supply on:

```bash
python python/scan_bus.py
```

✅ **Checkpoint (hand already has its IDs):**

```
ID   1 | position    -2.1 deg |  5.1 V |  28 C
...
ID   8 | position    +4.0 deg |  5.1 V |  28 C

Found 8 servo(s): [1, 2, 3, 4, 5, 6, 7, 8]
```

* All 8 found → your IDs were set during assembly. **Skip to 5.4** to double-check the mapping.
* Only `[1]` found although 8 servos are plugged → they all still have factory ID 1 (several
  servos answering at once scramble the reply). Continue with 5.2.
* Nothing → see [chapter 14](14_Troubleshooting.md#nothing-answers).

## 5.2 Setting IDs, method 1: Feetech FD software (Windows)

This is what the assembly guide uses.

1. Download *FD* (Feetech debug tool, e.g. FD1.9.8.2), linked in the repo's `PythonExample/README.md`.
2. Connect **one** new servo (or one finger = 2 servos, one at a time).
3. In FD: choose the COM port, BaudR = **1000000**, **Open**, then **Search**. The servo appears as `ID 1 SCS009`.
4. Tab **Programming** → the ID line (address 5) → type the new ID in the box at the bottom right → **Save**
   (see `assets/FD_IDs.jpg`).
5. Search again: it now appears with the new ID.

## 5.3 Setting IDs, method 2: `python/change_id.py`

Same thing from Python, so it also works on Linux/macOS.

```python
"""
change_id.py - give a servo a new ID.

!!! Connect ONLY the servo you want to change to the bus !!!
Every new SCS0009 ships with ID 1, so two new servos on the bus would both answer.
"""
import time

from rustypot import Scs0009PyController

SERIAL_PORT = "COM11"
BAUDRATE = 1_000_000
OLD_ID = 1   # a brand-new servo is always 1
NEW_ID = 2   # the ID you want (see the ID map in the tutorial)


def main():
    c = Scs0009PyController(serial_port=SERIAL_PORT, baudrate=BAUDRATE, timeout=0.1)

    try:
        c.ping(OLD_ID)
    except RuntimeError:
        raise SystemExit(f"No servo answers on ID {OLD_ID}. Run scan_bus.py first.")

    print(f"Changing ID {OLD_ID} -> {NEW_ID}")
    c.write_lock(OLD_ID, False)   # unlock the EEPROM so the new ID survives a power cycle
    time.sleep(0.05)
    c.write_id(OLD_ID, NEW_ID)    # from now on the servo answers on NEW_ID
    time.sleep(0.05)
    c.write_lock(NEW_ID, True)    # lock the EEPROM again
    time.sleep(0.05)

    print("Read back ID:", c.read_id(NEW_ID)[0])
    print("Done. Power-cycle the servo and run scan_bus.py to double check.")


if __name__ == "__main__":
    main()
```

* **Only one servo with ID `OLD_ID` may be on the bus.** Plug servos one at a time.
* `write_lock(id, False)` → writes 0 to register 48, which lifts the EEPROM write
  protection, so the new ID **survives a power cycle**. We lock again afterwards.
* After `write_id`, the servo only answers to `NEW_ID`, which is why the lock and the read-back use `NEW_ID`.

Procedure for a whole hand (servos one at a time):

| Plug servo … | set `NEW_ID` |
|---|---|
| index, motor 1 | 1 (already 1, nothing to do) |
| index, motor 2 | 2 |
| middle, motor 1 / 2 | 3 / 4 |
| ring, motor 1 / 2 | 5 / 6 |
| thumb, motor 1 / 2 | 7 / 8 |

Motor 1 / motor 2 are defined in `Demo/docs/finger.png`: in that view (servo horns facing
you, fingertip up) **motor 1 is on the right, motor 2 on the left.**

> The repo's Rust tool does the same: `cargo run --bin=change_id -- -s COM11 -o 1 -n 2` (chapter 11).

✅ **Checkpoint:** power-cycle, run `scan_bus.py`, and get `Found 8 servo(s): [1, 2, 3, 4, 5, 6, 7, 8]`.

## 5.4 Verify that every ID is where you think it is

Wrong IDs are the #1 cause of "the demo does weird things". Quick physical check with the
script from chapter 6 (`read_positions.py`, torque off): bend each finger by hand and watch
which two numbers change. **Closing** a finger should make **motor 1 go up (+)** and **motor 2 go down (−)**.
Write any swap down and fix it with `change_id.py` (use a temporary free ID like 30 to swap two servos).

Next → [06 Calibration](06_Calibration.md)
