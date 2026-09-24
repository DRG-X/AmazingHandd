# 14 · Troubleshooting & next steps

## Nothing answers

`scan_bus.py` finds nothing:

1. **Power**: is the 5 V supply plugged into the adapter? USB alone doesn't power the servos.
2. **Port name**: `python -m serial.tools.list_ports -v`. Unplug/replug the adapter and see which one appears.
3. **Jumper** on the Waveshare board in USB/PC mode (chapter 2).
4. **Port busy**: only one program can open a serial port. Close the FD software, Arduino serial
   monitor, another Python script, or a running dora dataflow.
5. **Linux permissions**: `Permission denied: '/dev/ttyACM0'` → `sudo usermod -aG dialout $USER`, log out/in.
6. **Linux, CH340 adapters vanishing** (`/dev/ttyUSB0` appears then disappears): Ubuntu's `brltty`
   grabs them. `sudo apt remove brltty`.
7. **Baudrate**: the servos must be at 1 000 000 (factory default). If someone changed it, find it
   with FD's search, which scans all baudrates.
8. **Cable orientation**: check GND/VCC/DATA on each connector.

## Only ID 1 answers, garbled data

Several servos have the same ID (factory ID 1). Set IDs with one servo on the bus at a time (chapter 5).

## `RuntimeError: ... Timeout` in the middle of a demo

* A loose connector (one finger's servos stop answering).
* Power supply too weak: the voltage sags when several servos start together and a servo browns
  out. Check `status` in `hand_cli.py` (voltage should stay ≥ 4.8 V) or use a 3 A supply.

## A finger buzzes / gets warm at the end of a move

It's pushing against a mechanical stop or another finger. Re-check calibration (chapter 6.3)
or reduce that pose's angles. The thumb crossing under the index in `CloseHand` is the classic
spot; the demo gives the thumb a higher speed so it gets there first.

## The finger tilts instead of closing (or closes instead of tilting)

Motor 1 / motor 2 swapped, or the two IDs belong to different fingers. Check with
`read_positions.py` (chapter 5.4): closing = motor 1 **+**, motor 2 **−**.

## The hand does something different than the Onshape named positions

Expected within a few degrees (printing tolerances, rod length adjustment, soft shells). The
repo README says so in its disclaimer. Fine-tune `MiddlePos` or adjust the pose values.

## Rust build errors

* `libudev` / `pkg-config` not found (Linux): `sudo apt install libudev-dev pkg-config`.
* Errors inside `dora-*` crates: the Rust toolchain is too old. Run `rustup update`.

## dora problems

* **Version mismatch** messages between CLI/daemon/node: all three must be 0.3.13
  (`dora --version`, `uv pip show dora-rs`, `Cargo.lock`).
* `pip: command not found` during `dora build`: you forgot `--uv` (the uv venv has no pip).
* Nodes can't connect / *"failed to open zenoh session"* with *"Address family not supported"*:
  your machine has IPv6 disabled. Create a file `zenoh.json5` with
  `{ listen: { endpoints: ["tcp/127.0.0.1:0"] } }` and set `ZENOH_CONFIG=/path/to/zenoh.json5`
  before `dora up` / `dora run`.
* Stale daemon after a crash: `dora destroy`, then `dora up` again.

## MuJoCo / MediaPipe

* No window / OpenGL error: update your GPU drivers; on a headless machine the viewer can't open.
* macOS: MuJoCo's passive viewer must run under `mjpython`. Use Linux for this demo.
* `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`: `uv pip install "numpy<2"`.
* Webcam not found: change `cv2.VideoCapture(0)` to `1`, `2`…
* Tracking jittery: good light, plain background, palm facing the camera.

## Next steps (ideas from the repo's to-do list)

1. **Gentle grasping:** close a finger in small steps, read `present_load`, stop when it rises
   (`amazing_hand.py` already exposes `read_status()`).
2. **Temperature watchdog:** a thread that releases torque if any servo exceeds ~60 °C.
3. **Record & replay:** torque off, move the fingers by hand, record `read_angles()` at 20 Hz, replay
   (the community project *AmazingHandControl*, linked in the README, does this with a GUI).
4. **New gestures:** design them in `hand_cli.py`, add them to `POSES_RIGHT`.
5. **Smoothing:** in the tracking demo, low-pass filter the fingertip vectors to reduce jitter.
6. Look at the **Amazing Hand Enhanced** branch of the original repo for the newer version.

Back to the [table of contents](README.md).
