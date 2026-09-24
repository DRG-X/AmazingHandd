# 10 · Advanced demo 1/4: architecture, tools, dora-rs

Goal: **the robot hand copies your hand**, live, from a webcam.

## 10.1 Architecture

```
 ┌──────────────┐ r_hand_pos  ┌─────────────────────┐ mj_r_joints_pos ┌──────────────┐   serial   ┌──────┐
 │ HandTracking │────────────►│ AHSimulation        │────────────────►│ AHControl    │──────────► │ hand │
 │ webcam +     │ 4 fingertip │ MuJoCo model + mink │ 8 motor angles  │ (Rust)       │  SYNC_WRITE└──────┘
 │ MediaPipe    │ vectors     │ inverse kinematics  │ + finger→index  │ + calibration│
 └──────────────┘  (20 Hz)    └─────────────────────┘   (100 Hz)      └──────────────┘
      Python                        Python                                  Rust
```

Why the simulation in the middle? The camera gives you **where the fingertips are**, but the
robot needs **motor angles**. Going from positions to angles through the parallel mechanism
(two motors → rods → gimbal → two phalanges) is an **inverse kinematics (IK)** problem.
The MuJoCo model contains the exact geometry, and `mink` solves the IK on it. As a bonus you
see the virtual hand on screen, so everything can be tested **without hardware**.

The 3 programs run at the same time and exchange data through **dora-rs**.

## 10.2 dora-rs in 5 minutes

* A **dataflow** is a YAML file listing **nodes** (programs).
* Each node declares **outputs** (names) and **inputs** (`input_name: other_node/output_name`).
* **Timers** are built-in inputs: `tick: dora/timer/millis/50` delivers an event every 50 ms.
* A node is a loop over events:

  ```python
  from dora import Node
  node = Node()
  for event in node:
      if event["type"] == "INPUT":
          event["id"]        # which input fired ("tick", "r_hand_pos", ...)
          event["value"]     # the data: an Apache Arrow array
          event["metadata"]  # dict of small extra parameters
          node.send_output("my_output", pyarrow_array, metadata_dict)
  ```

* Data is **Apache Arrow** (`pyarrow`). Nodes in different languages (Python, Rust) share it without copies.
* `build:` lines in the YAML are run by `dora build` (install / compile), `path:` is what `dora run` starts.
  Paths are relative to the YAML's folder.

## 10.3 Install the tools (Linux recommended)

The authors use Linux (`/dev/ttyACM0` everywhere). Windows should work but is less tested.
On macOS the MuJoCo viewer requires `mjpython`, which doesn't fit dora's launcher well.

```bash
# 1. Build tools (Ubuntu/Debian). libudev + pkg-config are needed by the Rust serialport crate
sudo apt install build-essential pkg-config libudev-dev git curl

# 2. Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env

# 3. uv (fast Python package manager, used by dora --uv)
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### 4. dora, pinned version (important!)

dora has moved on to 1.x, but this demo is written against **0.3.x**: the Rust node uses
`dora-node-api 0.3.13` and the Python nodes use `dora-rs 0.3.13`. The CLI/daemon **must be
the same version**, so do not install the latest dora. Install it inside the project's venv
(next section): `uv pip install dora-rs-cli==0.3.13`.

## 10.4 Create the `Demo/` skeleton

```
Demo/
├── Cargo.toml                          # Rust workspace (this chapter)
├── dataflow_*.yml                      # chapter 13
├── AHControl/                          # chapter 11
│   ├── Cargo.toml
│   ├── config/r_hand.toml, 2hands.toml
│   └── src/main.rs, src/bin/{change_id,goto,get_zeros,set_zeros}.rs
├── AHSimulation/                       # chapter 12
│   ├── pyproject.toml
│   ├── examples/finger_angle_control.py
│   └── AHSimulation/{__init__.py, hand_sim.py, mj_mink_right.py, mj_mink_left.py, AH_Right/, AH_Left/}
└── HandTracking/                       # chapter 13
    ├── pyproject.toml
    └── HandTracking/{__init__.py, main.py}
```

```bash
cd my_amazing_hand
mkdir -p Demo/AHControl/src/bin Demo/AHControl/config
mkdir -p Demo/AHSimulation/AHSimulation Demo/AHSimulation/examples
mkdir -p Demo/HandTracking/HandTracking
touch Demo/AHSimulation/AHSimulation/__init__.py Demo/HandTracking/HandTracking/__init__.py
```

### `Demo/Cargo.toml`: the Rust workspace

```toml
[workspace]
resolver = "2"
members = ["AHControl"]
```

A *workspace* groups Rust packages and shares one `target/` build folder at `Demo/target/`.
That's why the YAML runs `cargo build -p AHControl` and starts `target/debug/AHControl`.

### `Demo/AHSimulation/pyproject.toml`

```toml
[project]
name = "AHSimulation"
version = "0.1.0"
description = "dora node: MuJoCo model + mink inverse kinematics of the Amazing Hand"
requires-python = ">=3.12"
dependencies = [
    "dora-rs==0.3.13",
    "loop-rate-limiters>=1.1.2",
    "mink>=0.0.11",
    "mujoco>=3.3.2",
    "qpsolvers[quadprog]>=4.7.1",
    "scipy",                           # used by examples/finger_angle_control.py
]
```

### `Demo/HandTracking/pyproject.toml`

```toml
[project]
name = "HandTracking"
version = "0.1.0"
description = "dora node: webcam hand tracking with MediaPipe for the Amazing Hand"
requires-python = ">=3.9,<3.13"
dependencies = [
    "dora-rs==0.3.13",                 # same version as the dora CLI
    "mediapipe>=0.10.14,<=0.10.15",    # these versions still ship the mp.solutions.hands API
    "opencv-python",
    "numpy<2",                         # mediapipe 0.10.14/15 is built against numpy 1.x
]
```

Changes vs. the repo's pyproject files:

| Change | Why |
|---|---|
| `dora-rs==0.3.13` (repo: `>=0.3.11,<=0.3.13`) | must equal the CLI version |
| HandTracking `requires-python = "<3.13"` (repo: `<=3.12`) | `<=3.12` technically excludes 3.12.1+, plain pip refuses it (uv is lenient) |
| `numpy<2` in HandTracking | mediapipe 0.10.14/15 is a numpy-1 build |
| removed `readme` and `[project.scripts]` | the scripts pointed at modules that don't exist; dora runs files by path anyway |
| removed `onshape-to-robot` | only needed to regenerate the model from CAD (chapter 12.1) |
| added `scipy` to AHSimulation | the example imports it (it was only pulled in indirectly) |

## 10.5 The Python environment for the Demo

```bash
cd Demo
uv venv --python 3.12
source .venv/bin/activate            # Windows: .venv\Scripts\activate
uv pip install dora-rs-cli==0.3.13
dora --version                       # -> dora-cli 0.3.13
```

`dora build … --uv` will install the two local packages into this venv (the `build:` lines).

✅ **Checkpoint:** `dora --version` prints 0.3.13 and `cargo --version` works.

Next → [11 AHControl (Rust)](11_AHControl_Rust.md)
