# 11 · Advanced demo 2/4: AHControl, the Rust motor node

`AHControl` is one Rust package with **one dora node** (`src/main.rs`) and **four
command-line tools** (`src/bin/*.rs`). All of them use the Rust side of `rustypot`
(`Scs0009Controller`), which works like the Python one but with `Result`s instead of exceptions.

## 11.1 `Demo/AHControl/Cargo.toml`

```toml
[package]
name = "AHControl"
version = "0.1.0"
edition = "2021"

[dependencies]
clap = { version = "4.5.40", features = ["derive"] }  # command line arguments
dora-node-api = "=0.3.13"   # MUST match the dora CLI version you install
eyre = "0.6.12"             # error type
facet = "0.27.14"           # (de)serialization framework...
facet-pretty = "0.23.21"    # ...pretty printing
facet-toml = "0.25.15"      # ...TOML support
rustypot = "1.1.0"          # Feetech / Dynamixel servo driver
serialport = "4.7.2"        # serial port access
```

Differences with the repo: `dora-node-api` pinned to `=0.3.13` (the repo's `"0.3.11"` means
"≥0.3.11, <0.4", which today resolves to 0.3.13 anyway; pinning makes the CLI match
explicit), `clap` gets its `derive` feature explicitly (the repo only got it indirectly, because
rustypot enables it), and the two unused crates `arrow_convert` and `toml` are removed.

Every file in `src/bin/` becomes its own executable, runnable with
`cargo run --bin=<name> -- <args>`.

## 11.2 The config file: `Demo/AHControl/config/r_hand.toml`

It says, for each finger, which two motor IDs it uses and their **offsets**:

```toml
# AHControl config generated from MiddlePos = [3, 0, -5, -8, -2, 5, -12, 0] (degrees)

[[motors]]
finger_name = "r_finger1"  # index
motor1 = { id = 1, offset = 0.052360, invert = false, model = "SCS0009" }
motor2 = { id = 2, offset = 0.000000, invert = false, model = "SCS0009" }

[[motors]]
finger_name = "r_finger2"  # middle
motor1 = { id = 3, offset = -0.087266, invert = false, model = "SCS0009" }
motor2 = { id = 4, offset = -0.139626, invert = false, model = "SCS0009" }

[[motors]]
finger_name = "r_finger3"  # ring
motor1 = { id = 5, offset = -0.034907, invert = false, model = "SCS0009" }
motor2 = { id = 6, offset = 0.087266, invert = false, model = "SCS0009" }

[[motors]]
finger_name = "r_finger4"  # thumb
motor1 = { id = 7, offset = -0.209440, invert = false, model = "SCS0009" }
motor2 = { id = 8, offset = 0.000000, invert = false, model = "SCS0009" }
```

* `finger_name` must match the metadata keys the simulation sends (`r_finger1…4`,
  `l_finger1…4`): finger1 = index, 2 = middle, 3 = ring, 4 = thumb.
* `offset` (**radians**) = motor position when the simulated joint is 0. In the
  simulation, all motor joints at 0 = the **"Middle" pose**, the same reference as your Python
  calibration. So **offset = `MiddlePos` in radians**. (I checked this: in the MuJoCo model,
  bending a finger drives motor 1 positive and motor 2 negative, the same convention as the demos.)
* `invert`: negate the command (a motor mounted the other way). Leave it `false`.
  Caveat: the code computes `-(angle + offset)`, so if you ever use it, the offset
  must be negated too.
* The repo's file starts with a stray empty `[Fingers]` table; it's harmless and not needed.

You can write this file by hand, or generate it from your `MiddlePos` list with this helper:

### `python/middlepos_to_toml.py`

```python
"""
middlepos_to_toml.py - turn your Python calibration (degrees) into the AHControl TOML config (radians).

    python python/middlepos_to_toml.py > Demo/AHControl/config/r_hand.toml
"""
import math

SIDE = "r"                                   # "r" for right, "l" for left
MIDDLE_POS = [3, 0, -5, -8, -2, 5, -12, 0]   # <-- your calibration, degrees, IDs 1..8
IDS = [1, 2, 3, 4, 5, 6, 7, 8]               # IDs in the same order
FINGER_NAMES = ["index", "middle", "ring", "thumb"]

print(f"# AHControl config generated from MiddlePos = {MIDDLE_POS} (degrees)")
for k in range(4):
    print()
    print("[[motors]]")
    print(f'finger_name = "{SIDE}_finger{k + 1}"  # {FINGER_NAMES[k]}')
    for m in (1, 2):
        i = 2 * k + (m - 1)
        offset = math.radians(MIDDLE_POS[i])
        print(f'motor{m} = {{ id = {IDS[i]}, offset = {offset:.6f}, invert = false, model = "SCS0009" }}')
```

```bash
python python/middlepos_to_toml.py > Demo/AHControl/config/r_hand.toml
```

For two hands, `config/2hands.toml` simply has 8 `[[motors]]` blocks: `r_finger1-4` with IDs
1-8 and `l_finger1-4` with IDs 15/16 (index), 13/14, 11/12, 17/18 (thumb) in the repo.

## 11.3 The node: `Demo/AHControl/src/main.rs`

```rust
//! AHControl: dora node that receives joint angles from the simulation
//! and writes them to the real SCS0009 servos.
use clap::Parser;
use dora_node_api::{self, arrow::array::Array, DoraNode, Event, Parameter};
use eyre::{eyre, Result};
use facet::Facet;
use facet_pretty::FacetPretty;
use rustypot::servo;
use std::{error::Error, fs, thread, time::Duration};

// ---- Config file structure (mirrors config/r_hand.toml) --------------------
#[derive(Debug, Facet)]
struct Fingers {
    motors: Vec<Motors>,
}

#[derive(Debug, Facet)]
struct Motors {
    finger_name: String,
    motor1: Motor,
    motor2: Motor,
}

#[derive(Debug, Facet)]
struct Motor {
    id: u8,
    offset: f64, // radians: motor position when the simulated joint is at 0
    invert: bool,
    model: String,
}

// ---- Command line arguments ------------------------------------------------
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Serialport
    #[arg(short, long, default_value = "/dev/ttyACM0")]
    serialport: String,
    /// baudrate
    #[arg(short, long, default_value_t = 1_000_000)]
    baudrate: u32,
    /// TOML config file
    #[arg(short, long, default_value = "config/r_hand.toml")]
    config: String,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();

    // 1. Read and parse the TOML config
    println!("Opening {:?}", args.config);
    let toml_str = fs::read_to_string(&args.config).expect("Failed to read config file");
    let motors_conf: Fingers =
        facet_toml::from_str(&toml_str).expect("Failed to deserialize config file");
    println!("{}", motors_conf.pretty());

    // 2. Open the serial port and create the servo controller
    let serial_port = serialport::new(args.serialport, args.baudrate)
        .timeout(Duration::from_millis(10))
        .open()?;
    let mut controller = servo::feetech::scs0009::Scs0009Controller::new()
        .with_protocol_v1()
        .with_serial_port(serial_port);

    if motors_conf.motors[0].motor1.model != *"SCS0009" {
        return Err(eyre!("Only SCS0009 motors are supported for now...").into());
    };

    // 3. Flatten the config into parallel lists of IDs and offsets
    let motors = &motors_conf.motors;
    let mut motor_ids: Vec<u8> = vec![];
    let mut motor_offsets: Vec<f64> = vec![];
    for m in motors {
        motor_ids.push(m.motor1.id);
        motor_ids.push(m.motor2.id);
        motor_offsets.push(m.motor1.offset);
        motor_offsets.push(m.motor2.offset);
    }
    let motors_on: Vec<u8> = vec![1; motor_ids.len()];
    let motors_off: Vec<u8> = vec![0; motor_ids.len()];

    // 4. Torque on, then go to the "zero" pose (= the offsets)
    controller.sync_write_torque_enable(&motor_ids, &motors_on)?;
    thread::sleep(Duration::from_millis(1000));
    controller.sync_write_goal_position(&motor_ids, &motor_offsets)?;
    thread::sleep(Duration::from_millis(1000));

    // 5. Connect to dora and process events
    let (_node, mut events) = DoraNode::init_from_env()?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                "mj_l_joints_pos" | "mj_r_joints_pos" => {
                    // The payload is an Arrow Float64Array with 8 joint angles (radians)
                    let buffer: &dora_node_api::arrow::array::Float64Array =
                        data.as_any().downcast_ref().unwrap();
                    let buffer: &[f64] = buffer.values();

                    let mut ids: Vec<u8> = Vec::new();
                    let mut goals: Vec<f64> = Vec::new();

                    for finger in motors.iter() {
                        // metadata["r_finger1"] = [0, 1] tells us where this finger's
                        // two joint values are inside `buffer`
                        if let Some(Parameter::ListInt(idx)) =
                            metadata.parameters.get(&finger.finger_name)
                        {
                            let mut m1 = buffer[idx[0] as usize] + finger.motor1.offset;
                            if finger.motor1.invert {
                                m1 = -m1;
                            }
                            let mut m2 = buffer[idx[1] as usize] + finger.motor2.offset;
                            if finger.motor2.invert {
                                m2 = -m2;
                            }
                            ids.push(finger.motor1.id);
                            ids.push(finger.motor2.id);
                            goals.push(m1);
                            goals.push(m2);
                        }
                    }
                    // One packet moves all the motors of this message at once
                    controller.sync_write_goal_position(&ids, &goals)?;
                }
                other => println!("Received input `{other}`"),
            },
            Event::Stop(cause) => {
                eprintln!("Received stop: {:?}", cause);
                break; // (the original used `return`, which skipped the torque-off below)
            }
            _ => {}
        }
    }

    // 6. Release the motors
    println!("Quitting");
    controller.sync_write_torque_enable(&motor_ids, &motors_off)?;
    thread::sleep(Duration::from_millis(1000));
    Ok(())
}
```

Walkthrough:

1. **Config structs** + `#[derive(Facet)]`: `facet_toml::from_str` fills them from the TOML,
   field names = TOML keys. `pretty()` prints them nicely.
2. **`Args` with `clap`**: `--serialport` (`-s`), `--baudrate` (`-b`), `--config` (`-c`). The
   config path is relative to the **current directory**. dora runs nodes from the YAML folder,
   hence `--config AHControl/config/r_hand.toml` in the YAML.
3. **Serial port** opened with a 10 ms timeout, wrapped in `Scs0009Controller` with protocol v1
   (the Feetech SCS protocol from chapter 4).
4. **Flatten** the config into `motor_ids` / `motor_offsets` vectors.
5. **Startup:** torque on, then move to the offsets (the Middle pose) with one `sync_write`.
6. **`DoraNode::init_from_env()`** connects to the dora daemon (the daemon passes connection info
   through environment variables, so this binary only works when started by dora).
7. **Event loop:** for each `mj_r_joints_pos` / `mj_l_joints_pos` input:
   * downcast the Arrow array to `Float64Array` → `&[f64]` (8 angles, radians),
   * for each finger of the config, look up `metadata[finger_name]` = `[i, j]`, the positions of
     that finger's motor 1 / motor 2 angles in the array,
   * goal = angle + offset (negated if `invert`),
   * **one** `sync_write_goal_position` for all motors (one packet, all motors move together).
   * A message from the right hand only contains `r_finger*` keys, so with `2hands.toml`
     the same loop naturally handles both hands.
8. **Stop:** torque off. **Fixed:** the repo `return`ed inside `Event::Stop`, so the torque-off
   code after the loop never ran. We `break` instead.

## 11.4 The tools

### `src/bin/change_id.rs`: set a servo ID

```rust
//! change_id: give a servo a new ID.  cargo run --bin=change_id -- -s /dev/ttyACM0 -o 1 -n 2
use clap::Parser;
use rustypot::servo;
use std::{error::Error, thread, time::Duration};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Serialport
    #[arg(short, long, default_value = "/dev/ttyACM0")]
    serialport: String,
    /// baudrate
    #[arg(short, long, default_value_t = 1_000_000)]
    baudrate: u32,
    /// old id
    #[arg(short, long, default_value_t = 1)]
    old_id: u8,
    /// new id
    #[arg(short, long)]
    new_id: u8,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    println!("Opening port {} at baudrate {}", args.serialport, args.baudrate);
    println!("Changing id: {} into {}", args.old_id, args.new_id);

    let serial_port = serialport::new(&args.serialport, args.baudrate)
        .timeout(Duration::from_millis(10))
        .open()?;
    let mut controller = servo::feetech::scs0009::Scs0009Controller::new()
        .with_protocol_v1()
        .with_serial_port(serial_port);

    controller.write_id(args.old_id, args.new_id)?;
    thread::sleep(Duration::from_millis(1000));
    let id = controller.read_id(args.new_id)?;
    println!("Servo now answers on id {:?}", id);
    Ok(())
}
```

`cargo run --bin=change_id -- -s /dev/ttyACM0 -o 1 -n 2` (one servo on the bus!).
Unlike the Python version, it doesn't unlock the EEPROM first. If the new ID doesn't survive
a power cycle, use `change_id.py`.

### `src/bin/goto.rs`: move one motor

```rust
//! goto: move ONE motor to a position in radians.  cargo run --bin=goto -- -i 3 -p 0.5
use clap::Parser;
use rustypot::servo;
use std::{error::Error, thread, time::Duration};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Serialport
    #[arg(short, long, default_value = "/dev/ttyACM0")]
    serialport: String,
    /// baudrate
    #[arg(short, long, default_value_t = 1_000_000)]
    baudrate: u32,
    /// id
    #[arg(short, long, default_value_t = 1)]
    id: u8,
    /// pos (radians, 0 = middle)
    #[arg(short, long, default_value_t = 0.0, allow_hyphen_values = true)]
    pos: f64,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    println!("Opening port {} at baudrate {}", args.serialport, args.baudrate);
    println!("Moving motor ({}) to the pos: {}", args.id, args.pos);

    let serial_port = serialport::new(&args.serialport, args.baudrate)
        .timeout(Duration::from_millis(10))
        .open()?;
    let mut controller = servo::feetech::scs0009::Scs0009Controller::new()
        .with_protocol_v1()
        .with_serial_port(serial_port);

    let curpos = controller.read_present_position(args.id)?;
    println!("Current pos: {:?}", curpos);
    controller.write_torque_enable(args.id, 1)?;
    thread::sleep(Duration::from_millis(1000));
    controller.write_goal_position(args.id, args.pos)?;
    thread::sleep(Duration::from_millis(1000));
    println!("Quitting");
    Ok(())
}
```

`cargo run --bin=goto -- -s /dev/ttyACM0 -i 3 -p -0.5` (radians). I added
`allow_hyphen_values = true` so negative positions like `-p -0.5` parse.

### `src/bin/get_zeros.rs`: measure offsets by hand

```rust
//! get_zeros: motors go limp, you place the hand in the zero pose by hand,
//! press ENTER, and a new TOML config with measured offsets is printed.
use clap::Parser;
use eyre::{eyre, Result};
use facet::Facet;
use rustypot::servo;
use std::{error::Error, fs, io, thread, time::Duration};

#[derive(Debug, Facet)]
struct Fingers {
    motors: Vec<Motors>,
}

#[derive(Debug, Facet)]
struct Motors {
    finger_name: String,
    motor1: Motor,
    motor2: Motor,
}

#[derive(Debug, Facet)]
struct Motor {
    id: u8,
    offset: f64,
    invert: bool,
    model: String,
}

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Serialport
    #[arg(short, long, default_value = "/dev/ttyACM0")]
    serialport: String,
    /// baudrate
    #[arg(short, long, default_value_t = 1_000_000)]
    baudrate: u32,
    /// TOML config file
    #[arg(short, long, default_value = "config/r_hand.toml")]
    config: String,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    println!("Opening {:?}", args.config);
    let toml_str = fs::read_to_string(&args.config).expect("Failed to read config file");
    let mut motors_conf: Fingers =
        facet_toml::from_str(&toml_str).expect("Failed to deserialize config file");

    let serial_port = serialport::new(&args.serialport, args.baudrate)
        .timeout(Duration::from_millis(10))
        .open()?;
    let mut controller = servo::feetech::scs0009::Scs0009Controller::new()
        .with_protocol_v1()
        .with_serial_port(serial_port);

    if motors_conf.motors[0].motor1.model != *"SCS0009" {
        return Err(eyre!("Only SCS0009 motors are supported for now...").into());
    };

    let mut motor_ids: Vec<u8> = vec![];
    for m in &motors_conf.motors {
        motor_ids.push(m.motor1.id);
        motor_ids.push(m.motor2.id);
    }
    let motors_compliant: Vec<u8> = vec![2; motor_ids.len()]; // 2 = compliant in the original tool

    controller.sync_write_torque_enable(&motor_ids, &motors_compliant)?;
    thread::sleep(Duration::from_millis(1000));

    println!("Please type ENTER when zeros are done:");
    let mut input_string = String::new();
    while input_string != "\n" && input_string != "\r\n" {
        input_string.clear();
        io::stdin().read_line(&mut input_string).unwrap();
    }

    for m in motors_conf.motors.iter_mut() {
        let pos1 = controller.read_present_position(m.motor1.id)?;
        thread::sleep(Duration::from_millis(10));
        let pos2 = controller.read_present_position(m.motor2.id)?;
        thread::sleep(Duration::from_millis(10));
        m.motor1.offset = pos1[0];
        m.motor2.offset = pos2[0];
    }

    println!("New TOML config file:\n");
    println!("{}", facet_toml::to_string(&motors_conf)?);
    Ok(())
}
```

1. Sets all motors **compliant** (the repo writes value `2` to torque-enable for this),
2. you gently place every finger in the **Middle pose** (all motors 0°, see `assets/Named_Pos.jpg`),
3. press ENTER: it reads every position and **prints** a new TOML with the measured offsets.
   Copy it into `config/r_hand.toml`.

`cargo run --bin=get_zeros -- -s /dev/ttyACM0 -c config/r_hand.toml` (run from `Demo/AHControl/`).
This is an alternative to converting your Python calibration. It is quicker but less precise,
because holding 4 fingers exactly in pose by hand is hard.

### `src/bin/set_zeros.rs`: check the offsets

```rust
//! set_zeros: move every motor to its configured offset (the "zero" pose), then release.
use clap::Parser;
use eyre::{eyre, Result};
use facet::Facet;
use facet_pretty::FacetPretty;
use rustypot::servo;
use std::{error::Error, fs, thread, time::Duration};

#[derive(Debug, Facet)]
struct Fingers {
    motors: Vec<Motors>,
}

#[derive(Debug, Facet)]
struct Motors {
    finger_name: String,
    motor1: Motor,
    motor2: Motor,
}

#[derive(Debug, Facet)]
struct Motor {
    id: u8,
    offset: f64,
    invert: bool,
    model: String,
}

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Serialport
    #[arg(short, long, default_value = "/dev/ttyACM0")]
    serialport: String,
    /// baudrate
    #[arg(short, long, default_value_t = 1_000_000)]
    baudrate: u32,
    /// TOML config file
    #[arg(short, long, default_value = "config/r_hand.toml")]
    config: String,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    println!("Opening {:?}", args.config);
    let toml_str = fs::read_to_string(&args.config).expect("Failed to read config file");
    let motors_conf: Fingers =
        facet_toml::from_str(&toml_str).expect("Failed to deserialize config file");
    println!("{}", motors_conf.pretty());

    let serial_port = serialport::new(&args.serialport, args.baudrate)
        .timeout(Duration::from_millis(10))
        .open()?;
    let mut controller = servo::feetech::scs0009::Scs0009Controller::new()
        .with_protocol_v1()
        .with_serial_port(serial_port);

    if motors_conf.motors[0].motor1.model != *"SCS0009" {
        return Err(eyre!("Only SCS0009 motors are supported for now...").into());
    };

    let mut motor_ids: Vec<u8> = vec![];
    let mut motor_offsets: Vec<f64> = vec![];
    for m in &motors_conf.motors {
        motor_ids.push(m.motor1.id);
        motor_ids.push(m.motor2.id);
        motor_offsets.push(m.motor1.offset);
        motor_offsets.push(m.motor2.offset);
    }
    let motors_on: Vec<u8> = vec![1; motor_ids.len()];
    let motors_off: Vec<u8> = vec![0; motor_ids.len()];

    controller.sync_write_torque_enable(&motor_ids, &motors_on)?;
    thread::sleep(Duration::from_millis(1000));
    controller.sync_write_goal_position(&motor_ids, &motor_offsets)?;
    thread::sleep(Duration::from_millis(1000));
    println!("Quitting");
    controller.sync_write_torque_enable(&motor_ids, &motors_off)?;
    thread::sleep(Duration::from_millis(1000));
    Ok(())
}
```

Moves every motor to its offset (the Middle pose), then releases. Compare with the Middle
picture: fingers straight up, no sideways tilt.

## 11.5 Build and test

```bash
cd Demo
cargo build -p AHControl
cd AHControl
cargo run --bin=set_zeros -- -s /dev/ttyACM0 -c config/r_hand.toml
```

✅ **Checkpoint:** it compiles (Linux: if you see *"libudev not found"*, run
`sudo apt install libudev-dev pkg-config`) and `set_zeros` puts the hand in the Middle pose.

Next → [12 AHSimulation](12_AHSimulation.md)
