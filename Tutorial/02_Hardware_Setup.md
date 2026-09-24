# 02 · Hardware setup, wiring and safety

## 2.1 What you need on the desk

| Item | Notes |
|---|---|
| The assembled hand (8× SCS0009, all cables reaching the palm) | |
| **5 V power supply, 3 A recommended** (2 A minimum) | The project spec says 5 V / max 3 A. A 5 V 2 A wall adapter works for the demo; 8 servos stalling together can pull more. **Never** use the 7.4 V / 12 V supplies meant for bigger servos. |
| **Path A:** Waveshare *Bus Servo Adapter (A)* (or equivalent SCS/STS USB bus board) + USB cable | Talks to the PC over USB, powers the bus from its DC jack |
| **Path B:** Arduino (Mega recommended) + Feetech *TTLinker mini* + jumper wires | see chapter 9 |
| Feetech 3-pin servo extension cables / hub | to daisy-chain the servos |

## 2.2 Wiring (Path A, Waveshare adapter)

```
 wall 5V ──► [DC jack of the adapter]
                     │
 PC ──USB──► [Bus Servo Adapter] ──3-pin──► finger servos ──► ... ──► all 8 servos
                     (jumper on USB/PC mode)
```

1. **Jumper:** the Waveshare board has a jumper that selects who talks to the bus: the
   USB port (PC) or its header for an external microcontroller. Put it on the **USB/PC
   position** (labelled **A** on the Waveshare board; check the silkscreen/wiki of your board).
2. **Servo cables:** each SCS0009 has a 3-wire cable (GND, VCC, DATA). The servos are
   plugged in parallel on the bus (daisy chain or hub), so **order does not matter
   electrically**. Only IDs matter. Respect the connector orientation (GND to GND).
3. **Power:** plug the 5 V supply into the adapter. USB alone cannot power 8 servos.
4. **Common ground** is provided by the adapter. If you ever power the servos from a separate
   supply, connect its GND to the adapter's GND.

> **During ID setup (chapter 5) you will plug servos in one or two at a time.** Since all
> new servos have ID 1, it is much easier if the fingers' cables are still accessible.
> If your hand is already fully closed with shells, check chapter 5 first: your servos
> probably already have their IDs (they were set during assembly step 3).

## 2.3 First power-on check

* Plug the supply first, then USB.
* Nothing should move at power-on (torque is off by default).
* No smell, no heat. Touch the servos after a minute: they must stay cold.

## 2.4 Safety rules for the whole project

1. **Start slow.** Use low speeds (e.g. 3 rad/s) the first time you run anything new.
2. **Keep a hand on the power plug** the first time a script moves the hand. If something
   pushes against a mechanical stop and buzzes, **pull the power**.
3. **Never force a finger by hand when torque is on.** Run `read_positions.py`
   (chapter 6, torque off) if you want to move fingers manually.
4. **Watch temperatures** (`scan_bus.py` / `hand_cli.py status`). SCS0009 are small
   servos; holding a closed fist against an object heats them. Stay well below ~65 °C.
5. **Every script we write turns torque off (or opens the hand) on Ctrl+C.**
6. Stay inside **±90°** around the calibrated middle: that is the range the mechanism was designed for.

Next → [03 PC setup](03_PC_Setup.md)
