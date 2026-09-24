# 12 · Advanced demo 3/4: AHSimulation, MuJoCo + inverse kinematics

## 12.1 The MuJoCo model (copy it, don't type it)

Copy these two folders from the repo into your project:

```
Demo/AHSimulation/AHSimulation/AH_Right/   ->  same path in your project
Demo/AHSimulation/AHSimulation/AH_Left/    ->  same path in your project
```

They were **generated from the Onshape CAD** with `onshape-to-robot` (`config.json` is its
settings file). Regenerating them needs an Onshape API key and gives the same files, so copying
is the sensible choice. What is inside `mjcf/`:

| File | Content |
|---|---|
| `assets/*.stl` | 3D meshes of each part (binary) |
| `robot.xml` | the kinematic tree: bodies (parts), **joints**, **sites**, actuators, **equality constraints** |
| `joints_properties.xml`, `additional.xml` | extra defaults injected by onshape-to-robot |
| `scene.xml` | includes `robot.xml`, adds light, floor, and 4 **mocap** target spheres |
| `keyframes.xml` | the `"zero"` pose (all 68 `qpos` values) |

Concepts you need, all visible if you open `robot.xml` / `scene.xml`:

* **Joints**: 8 actuated hinges `finger{1..4}_motor{1,2}` (range ±90°) and many *passive* ones
  (`passive_ball*` for the ball joints, `passive2 (i)`, `passive4 (i)` … for the phalanges).
* **Closed loops**: a finger is a *parallel* mechanism (two rods meet at the finger), but a
  MuJoCo model is a tree. The loops are closed with **`<equality><connect site1 site2/>`**
  constraints that glue pairs of sites together (`closing_1 (1)_1` ↔ `closing_1 (1)_2`…).
* **Sites** `tip1…tip4`: frames attached to each distal phalanx, the points we want to control.
* **Mocap bodies** `finger{1..4}_target` (in `scene.xml`): kinematic bodies we can
  place anywhere (`data.mocap_pos`, `data.mocap_quat`), drawn as transparent red spheres. They are the
  *targets*. Mocap index `i` = `finger{i+1}_target`.
* Keyframe **`zero`**: all motors at 0 = the "Middle" pose.

## 12.2 Inverse kinematics with mink

`mink` solves "which joint velocities move my frames towards their targets" as a small
quadratic program (QP), weighting several **tasks**:

| Task | Cost | Meaning |
|---|---|---|
| `EqualityConstraintTask` | 1000 | keep the closed loops closed (rods keep their length) |
| `FrameTask(tip_i)` ×4 | 1.0 | tip `i` goes to target `i`, in **position** (tracking) *or* **orientation** (angle demo) |
| `PostureTask` | 0.01 | weak pull back to the zero pose, which keeps the solution unique and smooth |

Each step: `vel = mink.solve_ik(configuration, tasks, dt, "quadprog", damping)` then
`configuration.integrate_inplace(vel, dt)`. Repeated at 500 Hz, the joints converge to the targets.
The 8 motor joint angles are then exactly the servo commands (plus calibration offsets, chapter 11).

## 12.3 `Demo/AHSimulation/AHSimulation/hand_sim.py`

The repo has two 300-line files, `mj_mink_right.py` and `mj_mink_left.py`, that differ only by
the model folder, the `r_`/`l_` prefixes and 12 offset numbers. Here the shared code lives in
`hand_sim.py` and the two original files become tiny launchers, so the YAML files stay unchanged.

```python
"""
hand_sim.py - dora node: MuJoCo model of one Amazing Hand + mink inverse kinematics.

Inputs  (from the dataflow YAML):
  tick              every 2 ms  -> solve one IK step, refresh the viewer
  tick_ctrl         every 10 ms -> publish the 8 motor angles
  <s>_hand_pos      fingertip POSITION targets (hand tracking)      (mode "pos")
  <s>_hand_quat     fingertip ORIENTATION targets (angle example)   (mode "quat")
Output:
  mj_<s>_joints_pos 8 motor angles in radians + metadata telling which index is which finger
(<s> is "r" or "l")
"""
import os
import time
from pathlib import Path

import mink
import mujoco
import mujoco.viewer
import numpy as np
import pyarrow as pa
from dora import Node
from loop_rate_limiters import RateLimiter

ROOT_PATH = Path(os.path.dirname(os.path.abspath(__file__)))

MODEL_DIR = {"r": "AH_Right", "l": "AH_Left"}

# Hand tracking gives, for each finger, the vector base -> tip in a hand frame.
# We scale it by 1.5 (tuned by the authors) and add the position of the matching
# robot finger base (metres, in the model frame) to get a fingertip target.
SCALE = 1.5
TIP_OFFSETS = {
    "r": [(-0.025, 0.022, 0.098), (-0.025, -0.009, 0.092), (-0.025, -0.040, 0.082), (0.024, 0.019, 0.017)],
    "l": [(0.025, -0.022, 0.098), (0.025, 0.009, 0.092), (0.025, 0.040, 0.082), (0.024, -0.019, 0.017)],
}

MOTOR_JOINTS = [f"finger{f}_motor{m}" for f in range(1, 5) for m in (1, 2)]


class Client:
    def __init__(self, side="r", mode="pos"):
        if mode not in ("pos", "quat"):
            raise ValueError(f"unknown mode: {mode}")
        self.side = side
        self.model = mujoco.MjModel.from_xml_path(
            str(ROOT_PATH / MODEL_DIR[side] / "mjcf" / "scene.xml")
        )
        self.configuration = mink.Configuration(self.model)

        # Task 1: stay close to the initial posture (small cost = weak preference)
        self.posture_task = mink.PostureTask(self.model, cost=1e-2)

        # Tasks 2-5: each fingertip site follows its mocap target,
        # either in position (tracking) or in orientation (angle control)
        pos_cost, ori_cost = (1.0, 0.0) if mode == "pos" else (0.0, 1.0)
        self.tip_tasks = [
            mink.FrameTask(
                frame_name=f"tip{i}",
                frame_type="site",
                position_cost=pos_cost,
                orientation_cost=ori_cost,
                lm_damping=1.0,
            )
            for i in range(1, 5)
        ]

        # Task 6: keep the closed kinematic loops (ball-joint rods) closed. High cost.
        eq_task = mink.EqualityConstraintTask(self.model, cost=1000.0)

        self.tasks = [eq_task, self.posture_task, *self.tip_tasks]

        self.model = self.configuration.model
        self.data = self.configuration.data
        self.solver = "quadprog"

        self.motor_pos = np.zeros(8)
        # Tells the Rust controller where each finger's 2 values are in motor_pos
        self.metadata = {f"{side}_finger{i}": [2 * (i - 1), 2 * (i - 1) + 1] for i in range(1, 5)}
        self.node = Node()

    def run(self):
        s = self.side
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            rate = RateLimiter(frequency=1000.0)   # only used for its dt (1 ms)

            # Start from the "zero" keyframe = every motor at 0 = the "Middle" pose
            self.configuration.update_from_keyframe("zero")
            self.posture_task.set_target_from_configuration(self.configuration)
            for i in range(1, 5):
                mink.move_mocap_to_frame(self.model, self.data, f"finger{i}_target", f"tip{i}", "site")

            for event in self.node:
                if event["type"] == "ERROR":
                    raise ValueError("An error occurred in the dataflow: " + event["error"])
                if event["type"] != "INPUT":
                    continue
                event_id = event["id"]

                if event_id == "tick":
                    if not viewer.is_running():
                        break
                    step_start = time.time()

                    # 1. targets = where the mocap spheres are
                    for i, task in enumerate(self.tip_tasks, start=1):
                        task.set_target(mink.SE3.from_mocap_name(self.model, self.data, f"finger{i}_target"))

                    # 2. one differential-IK step: solve for joint velocities, integrate them
                    vel = mink.solve_ik(self.configuration, self.tasks, rate.dt, self.solver, 1e-5)
                    self.configuration.integrate_inplace(vel, rate.dt)

                    # 3. read the 8 motor joint angles
                    self.motor_pos = np.array([self.data.joint(name).qpos[0] for name in MOTOR_JOINTS])

                    viewer.sync()
                    time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)

                elif event_id == "tick_ctrl":
                    self.node.send_output(f"mj_{s}_joints_pos", pa.array(self.motor_pos), self.metadata)

                elif event_id == f"{s}_hand_pos":
                    self.write_mocap_pos(event["value"])

                elif event_id == f"{s}_hand_quat":
                    self.write_mocap_quat(event["value"])

    def write_mocap_pos(self, hand):
        """hand = Arrow array holding one struct {"r_tip1": [x,y,z], ...}"""
        for i in range(4):
            key = f"{self.side}_tip{i + 1}"
            if key in hand[0]:
                x, y, z = hand[0][key].values.to_pylist()
                ox, oy, oz = TIP_OFFSETS[self.side][i]
                self.data.mocap_pos[i] = [x * SCALE + ox, y * SCALE + oy, z * SCALE + oz]

    def write_mocap_quat(self, hand):
        """hand = Arrow array holding one struct {"r_tip1": [w,x,y,z], ...}"""
        for i in range(4):
            key = f"{self.side}_tip{i + 1}"
            if key in hand[0]:
                self.data.mocap_quat[i] = hand[0][key].values.to_pylist()
```

Walkthrough:

* **`__init__`**: load `scene.xml`, create the mink `Configuration`, the 6 tasks described above.
  `mode="pos"`: position cost 1, orientation 0 (tracking). `mode="quat"`: the opposite (angle demo).
  `metadata` = `{"r_finger1": [0, 1], …}` tells AHControl where each finger's pair of values sits
  in the 8-value array.
* **`run`**: opens the passive viewer (you watch, the code drives), loads the `zero`
  keyframe, puts the mocap spheres on the fingertips, then loops over dora events:
  * `tick` (2 ms): targets ← mocap poses; one IK step; read the 8 motor joints; refresh the viewer.
  * `tick_ctrl` (10 ms): publish `mj_r_joints_pos` = the 8 angles + metadata (100 Hz to the motors).
  * `r_hand_pos` (from the tracker): move the mocap targets (`write_mocap_pos`).
  * `r_hand_quat` (from the example): set the mocap orientations (`write_mocap_quat`).
  * Closing the viewer window ends the node.
* **`write_mocap_pos`**: tracker vectors (metres, base→tip of each human finger) × 1.5
  + the position of the robot finger's base. `TIP_OFFSETS` are the repo's constants, unchanged.
* The repo's empty handlers (`pull_position`, `pull_velocity`, …) and commented code are dropped.

### `mj_mink_right.py` and `mj_mink_left.py`

```python
"""dora node: simulated RIGHT hand. Usage in the YAML: path: AHSimulation/AHSimulation/mj_mink_right.py"""
import argparse

from hand_sim import Client


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mode", choices=["pos", "quat"], default="pos",
                        help="pos = control fingertip positions, quat = control fingertip orientations")
    args = parser.parse_args()
    Client(side="r", mode=args.mode).run()


if __name__ == "__main__":
    main()
```

`mj_mink_left.py` is identical except the docstring and `side="l"`.

## 12.4 `Demo/AHSimulation/examples/finger_angle_control.py`

A fake "brain" that makes the fingers wave, useful to test the simulation without a camera.

```python
"""
finger_angle_control.py - dora node that makes the simulated fingers wave.

Every tick it sends ONE Arrow struct with a target ORIENTATION (quaternion w,x,y,z)
for each distal phalanx of both hands: {"r_tip1": q, ..., "l_tip4": q}.
The simulation nodes run in "quat" mode and solve the IK to reach them.
"""
import time

import numpy as np
import pyarrow as pa
from dora import Node
from scipy.spatial.transform import Rotation

# Useful numbers from the authors:
#   motors at 0 deg  => distal phalanx pitched ~121.9 deg in the finger base frame
#   flexion range (distal phalanx) [0, 140] deg,  abduction range [-20, 20] deg


def quat(seq, angles):
    """Euler angles (radians) -> quaternion, scalar first (w, x, y, z) like MuJoCo."""
    return Rotation.from_euler(seq, angles).as_quat(scalar_first=True)


def main():
    node = Node()
    t0 = time.time()

    for event in node:
        if event["type"] == "ERROR":
            raise RuntimeError(event["error"])
        if event["type"] != "INPUT" or event["id"] != "tick":
            continue

        t = time.time() - t0
        wave = np.sin(2.0 * np.pi * 1.0 * t)        # 1 Hz, in [-1, 1]
        wave_c = np.cos(2.0 * np.pi * 1.0 * t)

        s1_pitch = wave * np.radians(10.0) + np.radians(10.0)             # finger 1: small flexion 0..20 deg
        r_s1_roll = wave_c * np.radians(10.0)                              # ... plus sideways motion
        l_s1_roll = wave_c * np.radians(-10.0)                             # (mirrored on the left hand)
        s2_pitch = wave * np.radians(140.0 / 2) + np.radians(140.0 / 2)   # fingers 2,3: full 0..140 deg
        s4_pitch = wave * np.radians((90.0 + 53.0) / 2) + np.radians((90.0 - 53.0) / 2)  # thumb

        targets = {
            "r_tip1": quat("XYZ", [r_s1_roll, s1_pitch, 0.0]),
            "l_tip1": quat("XYZ", [l_s1_roll, s1_pitch, 0.0]),
            "r_tip2": quat("XYZ", [np.radians(10.0), s2_pitch, 0.0]),    # finger 2 is mounted with a 10 deg roll
            "l_tip2": quat("XYZ", [np.radians(-10.0), s2_pitch, 0.0]),
            "r_tip3": quat("XYZ", [np.radians(20.0), s2_pitch, 0.0]),    # finger 3: 20 deg roll
            "l_tip3": quat("XYZ", [np.radians(-20.0), s2_pitch, 0.0]),
            "r_tip4": quat("xyz", [0.0, -s4_pitch, np.radians(20.0)]),   # thumb: 20 deg yaw
            "l_tip4": quat("xyz", [0.0, -s4_pitch, np.radians(-20.0)]),
        }
        node.send_output("hand_quat", pa.array([targets]))


if __name__ == "__main__":
    main()
```

* Sends **orientations** (quaternions `w, x, y, z`, MuJoCo's order, hence `scalar_first=True`)
  for the 4 distal phalanges of both hands, at 20 Hz.
* Each is built from Euler angles: *pitch* = flexion, *roll* = abduction, plus the fixed roll/yaw
  at which fingers 2, 3 and the thumb are mounted on the palm.
* Everything goes in one Arrow struct array: `pa.array([{"r_tip1": q, …}])`.

## 12.5 Run the angle demo (no hardware)

```bash
cd Demo
source .venv/bin/activate
dora up                                   # starts the dora coordinator + daemon
dora build dataflow_angle_simu.yml --uv   # once: installs AHSimulation into the venv
dora run dataflow_angle_simu.yml --uv
```

(`dataflow_angle_simu.yml` is written in chapter 13.1. Type it now if you want to run this.)

✅ **Checkpoint:** two MuJoCo windows open and the fingers of both hands wave smoothly.
Ctrl+C in the terminal stops everything. `dora destroy` stops the daemon.

Next → [13 HandTracking + running everything](13_HandTracking_And_Run.md)
