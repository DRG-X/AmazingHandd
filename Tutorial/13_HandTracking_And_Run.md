# 13 · Advanced demo 4/4: HandTracking + running everything

## 13.1 The dataflows

Type these four files in `Demo/`. They are the repo's files with comments, plus one fix in
`dataflow_tracking_simu.yml`.

### `Demo/dataflow_angle_simu.yml`: wave demo, simulation only

```yaml
# Both simulated hands follow the finger_angle_control.py wave. No hardware needed.
nodes:
  - id: move_angle
    build: pip install -e AHSimulation
    path: AHSimulation/examples/finger_angle_control.py
    inputs:
      tick: dora/timer/millis/50        # 20 Hz
    outputs:
      - hand_quat

  - id: hand_simulation_r
    build: pip install -e AHSimulation
    path: AHSimulation/AHSimulation/mj_mink_right.py
    args: -m quat                       # orientation targets
    inputs:
      r_hand_quat: move_angle/hand_quat
      tick: dora/timer/millis/2         # IK + viewer at 500 Hz
      tick_ctrl: dora/timer/millis/10   # publish motor angles at 100 Hz
    outputs:
      - mj_r_joints_pos

  - id: hand_simulation_l
    build: pip install -e AHSimulation
    path: AHSimulation/AHSimulation/mj_mink_left.py
    args: -m quat
    inputs:
      l_hand_quat: move_angle/hand_quat
      tick: dora/timer/millis/2
      tick_ctrl: dora/timer/millis/10
    outputs:
      - mj_l_joints_pos
```

Read it as a wiring diagram: `move_angle` publishes `hand_quat` every 50 ms; both simulation
nodes receive it as `r_hand_quat` / `l_hand_quat` (the *input* name is what the node sees in
`event["id"]`). `args: -m quat` is passed to the Python script (`argparse`).

### `Demo/dataflow_tracking_simu.yml`: webcam, simulation only

```yaml
# Webcam -> both simulated hands. No hardware needed.
nodes:
  - id: hand_tracker
    build: pip install -e HandTracking
    path: HandTracking/HandTracking/main.py
    inputs:
      tick: dora/timer/millis/50
    outputs:
      - r_hand_pos
      - l_hand_pos

  - id: r_hand_simulation
    build: pip install -e AHSimulation
    path: AHSimulation/AHSimulation/mj_mink_right.py
    inputs:
      r_hand_pos: hand_tracker/r_hand_pos
      tick: dora/timer/millis/2
      tick_ctrl: dora/timer/millis/10
    outputs:
      - mj_r_joints_pos                 # (the repo wrote "mj_joints_pos": dora ignored the real output)

  - id: l_hand_simulation
    build: pip install -e AHSimulation
    path: AHSimulation/AHSimulation/mj_mink_left.py
    inputs:
      l_hand_pos: hand_tracker/l_hand_pos
      tick: dora/timer/millis/2
      tick_ctrl: dora/timer/millis/10
    outputs:
      - mj_l_joints_pos
```

**Fixed:** the repo declared the output as `mj_joints_pos` while the nodes send
`mj_r_joints_pos` / `mj_l_joints_pos`. dora silently drops outputs that are not declared
(it only logs a warning), which is harmless here because nothing listens, but it's wrong.

### `Demo/dataflow_tracking_real.yml`: webcam → real right hand

```yaml
# Webcam -> simulated right hand -> REAL right hand.
nodes:
  - id: hand_tracker
    build: pip install -e HandTracking
    path: HandTracking/HandTracking/main.py
    inputs:
      tick: dora/timer/millis/50
    outputs:
      - r_hand_pos

  - id: r_hand_simulation
    build: pip install -e AHSimulation
    path: AHSimulation/AHSimulation/mj_mink_right.py
    inputs:
      r_hand_pos: hand_tracker/r_hand_pos
      tick: dora/timer/millis/2
      tick_ctrl: dora/timer/millis/10
    outputs:
      - mj_r_joints_pos

  - id: hand_controller
    build: cargo build -p AHControl
    path: target/debug/AHControl        # add .exe on Windows
    args: --serialport /dev/ttyACM0 --config AHControl/config/r_hand.toml   # <-- your port
    inputs:
      mj_r_joints_pos: r_hand_simulation/mj_r_joints_pos
```

### `Demo/dataflow_tracking_real_2hands.yml`: webcam → two real hands

```yaml
# Webcam -> both simulated hands -> both REAL hands on one bus (IDs 1-8 and 11-18).
nodes:
  - id: hand_tracker
    build: pip install -e HandTracking
    path: HandTracking/HandTracking/main.py
    inputs:
      tick: dora/timer/millis/50
    outputs:
      - r_hand_pos
      - l_hand_pos

  - id: r_hand_simulation
    build: pip install -e AHSimulation
    path: AHSimulation/AHSimulation/mj_mink_right.py
    inputs:
      r_hand_pos: hand_tracker/r_hand_pos
      tick: dora/timer/millis/2
      tick_ctrl: dora/timer/millis/10
    outputs:
      - mj_r_joints_pos

  - id: l_hand_simulation
    build: pip install -e AHSimulation
    path: AHSimulation/AHSimulation/mj_mink_left.py
    inputs:
      l_hand_pos: hand_tracker/l_hand_pos
      tick: dora/timer/millis/2
      tick_ctrl: dora/timer/millis/10
    outputs:
      - mj_l_joints_pos

  - id: hand_controller
    build: cargo build -p AHControl
    path: target/debug/AHControl
    args: --serialport /dev/ttyACM0 --config AHControl/config/2hands.toml
    inputs:
      mj_r_joints_pos: r_hand_simulation/mj_r_joints_pos
      mj_l_joints_pos: l_hand_simulation/mj_l_joints_pos
```

## 13.2 How hand tracking works

MediaPipe Hands finds **21 landmarks** per hand in an image:

```
 0 WRIST
 1-4   THUMB  (CMC, MCP, IP, TIP)
 5-8   INDEX  (MCP, PIP, DIP, TIP)
 9-12  MIDDLE (MCP, PIP, DIP, TIP)
13-16  RING   (MCP, PIP, DIP, TIP)
17-20  PINKY  (MCP, PIP, DIP, TIP)
```

It returns them twice: `multi_hand_landmarks` (normalized image coordinates) and
`multi_hand_world_landmarks` (**metres**, centred on the hand), plus `multi_handedness`
(`"Left"`/`"Right"` + a confidence score).

The robot has no pinky, so we map human index/middle/ring/thumb → robot finger 1/2/3/4.
For each finger we compute the vector **MCP (base) → TIP** in metres. If you move your
whole hand, those vectors rotate with it, and we only want the finger bending. So they are
expressed in a **palm frame**:

* **z**: wrist → middle-finger base (along the palm),
* **x**: perpendicular to the palm (cross product of wrist → pinky base with z),
* **y**: completes the frame.

`R` has those axes as rows, so `R @ v` gives `v` in palm coordinates. The simulation then scales
these vectors (×1.5) and places them on the robot fingers' bases (chapter 12).

## 13.3 `Demo/HandTracking/HandTracking/main.py`

```python
"""
HandTracking node: webcam -> MediaPipe hand landmarks -> fingertip vectors.

On every "tick" it grabs a frame, finds the hands and sends, for each hand seen,
one Arrow struct {"r_tip1": [x,y,z], ..., "r_tip4": [x,y,z]} on output r_hand_pos
(or l_hand_pos). Each vector goes from the base (MCP) of a finger to its tip,
expressed in a frame attached to the palm, so it does not change when you
move or rotate your whole hand. Only the finger bending changes it.
"""
import cv2
import mediapipe as mp
import numpy as np
import pyarrow as pa
from dora import Node

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
LM = mp_hands.HandLandmark

# Robot finger 1..4 = human index, middle, ring, thumb. (tip, base) landmark pairs:
FINGERS = [
    (LM.INDEX_FINGER_TIP, LM.INDEX_FINGER_MCP),
    (LM.MIDDLE_FINGER_TIP, LM.MIDDLE_FINGER_MCP),
    (LM.RING_FINGER_TIP, LM.RING_FINGER_MCP),
    (LM.THUMB_TIP, LM.THUMB_MCP),
]


def point(landmarks, idx):
    p = landmarks.landmark[idx]
    return np.array([p.x, p.y, p.z])


def palm_rotation(norm_landmarks, label):
    """Rotation matrix whose rows are the axes of a palm frame:
    z: wrist -> base of the middle finger
    x: normal to the palm
    y: completes the frame (sign flipped because the image is mirrored)"""
    origin = point(norm_landmarks, LM.WRIST)
    unit_z = point(norm_landmarks, LM.MIDDLE_FINGER_MCP) - origin
    unit_z /= np.linalg.norm(unit_z)
    if label == "Right":
        towards_y = point(norm_landmarks, LM.PINKY_MCP) - origin
    else:
        towards_y = point(norm_landmarks, LM.INDEX_FINGER_MCP) - origin
    unit_x = np.cross(towards_y, unit_z)
    unit_x /= np.linalg.norm(unit_x)
    unit_y = np.cross(unit_z, unit_x)
    return np.array([unit_x, -unit_y, unit_z])


def process_img(hand_proc, image):
    image.flags.writeable = False
    results = hand_proc.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    image.flags.writeable = True

    r_res, l_res = None, None
    if not results.multi_hand_landmarks:
        return image, r_res, l_res

    for index, handedness in enumerate(results.multi_handedness):
        classif = handedness.classification[0]
        if classif.score <= 0.8:                                   # ignore uncertain detections
            continue
        world = results.multi_hand_world_landmarks[index]          # metric, origin near the hand centre
        norm = results.multi_hand_landmarks[index]                 # normalized image coordinates

        mp_drawing.draw_landmarks(
            image, norm, mp_hands.HAND_CONNECTIONS,
            mp_drawing_styles.get_default_hand_landmarks_style(),
            mp_drawing_styles.get_default_hand_connections_style(),
        )

        R = palm_rotation(norm, classif.label)
        tips = [R @ (point(world, tip) - point(world, base)) for tip, base in FINGERS]

        if classif.label == "Right":
            r_res = [{f"r_tip{i + 1}": tips[i] for i in range(4)}]
        elif classif.label == "Left":
            l_res = [{f"l_tip{i + 1}": tips[i] for i in range(4)}]
    return image, r_res, l_res


def main():
    node = Node()
    cap = cv2.VideoCapture(0)          # 0 = first webcam; try 1, 2... if you have several

    with mp_hands.Hands(model_complexity=0,
                        min_detection_confidence=0.5,
                        min_tracking_confidence=0.5) as hands:
        for event in node:
            if event["type"] == "ERROR":
                raise RuntimeError(event["error"])
            if event["type"] != "INPUT" or event["id"] != "tick":
                continue

            ret, frame = cap.read()
            if not ret:
                continue
            frame = cv2.flip(frame, 1)   # mirror view: MediaPipe's Right/Left labels then match your hands
            frame, r_res, l_res = process_img(hands, frame)

            if r_res is not None:
                node.send_output("r_hand_pos", pa.array(r_res))
            if l_res is not None:
                node.send_output("l_hand_pos", pa.array(l_res))

            cv2.imshow("MediaPipe Hands", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cap.release()


if __name__ == "__main__":
    main()
```

Walkthrough:

* `FINGERS`: (tip, base) landmark pairs, replacing the repo's 12 copy-pasted `tipN_x/y/z` lines.
* `palm_rotation`: identical maths to the repo's inline code (I checked numerically that the
  matrices match). Like the repo, it builds the frame from the *normalized* landmarks and applies
  it to the *metric* vectors.
* `process_img`: BGR→RGB (OpenCV images are BGR, MediaPipe wants RGB), detect, skip detections
  with confidence ≤ 0.8, draw the skeleton, compute the 4 vectors, pack them as
  `[{"r_tip1": v1, …}]`, one struct per hand.
* `main`: on each 50 ms `tick`, grab a webcam frame, **mirror it** (`cv2.flip(frame, 1)`), which
  makes the preview behave like a mirror and makes MediaPipe's Left/Right labels match your real
  hands, then process, send, show. Press `q` in the preview window to stop the tracker.
* Removed from the repo version: unused imports (`argparse`, `os`, `time`, `scipy`) and ~60
  lines of commented-out experiments.

## 13.4 Run it: the full sequence

Always go **simulation first, then real hardware**.

```bash
cd Demo
source .venv/bin/activate
dora up

# 1. Build once (installs the Python packages, compiles the Rust node)
dora build dataflow_tracking_real.yml --uv

# 2. Simulation only: check the virtual hand follows yours
dora run dataflow_tracking_simu.yml --uv
```

✅ **Checkpoint A:** a webcam window shows your hand skeleton; a MuJoCo window shows the
robot hand; when you close your fingers the red spheres move and the virtual fingers follow.
Keep your palm facing the camera, fingers up, about 50 cm away.

```bash
# 3. Real hardware: check the port in the YAML (args: --serialport ...) and your r_hand.toml
#    Hand powered, on its base, nothing in the way
dora run dataflow_tracking_real.yml --uv
```

✅ **Checkpoint B:** at start the real hand goes to the Middle pose (AHControl startup), then it
copies your movements, like the video in the repo README. Ctrl+C stops the dataflow and AHControl
releases the torque.

If the real hand moves but a finger goes the wrong way, or is offset, compare with the
simulation: the simulation is right by construction, so the difference comes from the TOML
(wrong ID ↔ finger, offset, or `invert`).

## 13.5 Two hands

Needs both hands on one bus with IDs 1-8 and 11-18 and `config/2hands.toml` with **your**
offsets. Check which left finger each ID pair moves (chapter 8.3 warning) and set the
`finger_name`s accordingly. Then:

```bash
dora build dataflow_tracking_real_2hands.yml --uv
dora run dataflow_tracking_real_2hands.yml --uv
```

🎉 That's the full project: from an empty folder to a hand that copies yours.

Next → [14 Troubleshooting & next steps](14_Troubleshooting.md)
