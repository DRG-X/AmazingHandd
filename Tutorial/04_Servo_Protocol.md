# 04 · Under the hood: the Feetech serial protocol

*Optional, but after this chapter `rustypot` is no longer a black box.* We write a
~150-line driver with nothing but `pyserial`, then check it against a real servo.

## 4.1 Packets

Every exchange is: the PC sends an **instruction packet**, the servo addressed answers
with a **status packet**.

```
Instruction:  FF FF | ID | LEN | INSTR | P1 ... Pn | CHK
Status:       FF FF | ID | LEN | ERROR | P1 ... Pn | CHK

LEN = n + 2            (instruction/error byte + params + checksum)
CHK = ~(ID + LEN + INSTR + P1 + ... + Pn) & 0xFF
```

| INSTR | Name | Params |
|---|---|---|
| 0x01 | PING | none |
| 0x02 | READ | address, number of bytes |
| 0x03 | WRITE | address, data bytes… |
| 0x04 | REG_WRITE | like WRITE, but only executed on ACTION |
| 0x05 | ACTION | (broadcast) execute all REG_WRITEs now |
| 0x83 | SYNC_WRITE | address, length, then `[ID, data…]` for each servo: one packet for many servos, no answer |

ID `0xFE` = broadcast (all servos, nobody answers).

### Worked example: "servo 1, go to +90°"

* +90° → raw = 511 + 90/0.293 ≈ **818 = 0x0332**
* Goal position is register **42 = 0x2A**, 2 bytes, **big-endian** → `03 32`
* Params: `2A 03 32` → n = 3 → LEN = 5
* Sum = 01 + 05 + 03 + 2A + 03 + 32 = 0x68 → CHK = ~0x68 & 0xFF = **0x97**

```
FF FF 01 05 03 2A 03 32 97
```

That is literally what travels on the wire. The code below prints it so you can check.

## 4.2 Half-duplex and echo

There is only one data wire. Some adapters feed what you send back into your RX line
(**echo**). Our reader therefore skips a received packet that is byte-for-byte identical to
the one we just sent.

## 4.3 The file: `python/raw_protocol.py`

```python
"""
raw_protocol.py - a tiny Feetech SCS driver written from scratch with pyserial.

The goal is to learn what really travels on the wire. The rest of the
project uses the `rustypot` library, which does the same thing, only faster
and with more features.
"""
import time

import serial  # pip install pyserial

# ---------------------------------------------------------------------------
# 1. Protocol constants
# ---------------------------------------------------------------------------
HEADER = bytes([0xFF, 0xFF])

# Instruction codes
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

# SCS0009 register addresses (memory table)
ADDR_ID = 5
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42      # 2 bytes
ADDR_GOAL_SPEED = 46         # 2 bytes
ADDR_PRESENT_POSITION = 56   # 2 bytes
ADDR_PRESENT_VOLTAGE = 62    # 1 byte, unit = 0.1 V
ADDR_PRESENT_TEMPERATURE = 63  # 1 byte, unit = deg C

# Position encoding: 1024 steps span 300 degrees, 511 is the middle.
STEPS_PER_DEGREE = 1024 / 300   # about 3.41 steps per degree (1 step = 0.293 deg)
CENTER = 511


# ---------------------------------------------------------------------------
# 2. Building packets
# ---------------------------------------------------------------------------
def checksum(body):
    """Feetech checksum: bitwise NOT of the sum of ID, LENGTH, INSTR, PARAMS (low byte)."""
    return (~sum(body)) & 0xFF


def build_packet(servo_id, instruction, params=()):
    """FF FF | ID | LENGTH | INSTRUCTION | PARAM_1 ... PARAM_N | CHECKSUM"""
    length = len(params) + 2          # LENGTH counts the instruction + params + checksum
    body = [servo_id, length, instruction, *params]
    return HEADER + bytes(body) + bytes([checksum(body)])


def degrees_to_raw(deg):
    raw = int(round(CENTER + deg * STEPS_PER_DEGREE))
    return max(0, min(1023, raw))


def raw_to_degrees(raw):
    return (raw - CENTER) / STEPS_PER_DEGREE


# ---------------------------------------------------------------------------
# 3. Talking to the bus
# ---------------------------------------------------------------------------
class ScsBus:
    def __init__(self, port, baudrate=1_000_000, timeout=0.02):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    def close(self):
        self.ser.close()

    def _read_packet(self):
        """Read one packet from the bus. Returns (raw_bytes, id, error, params) or None on timeout."""
        prev = None
        while True:                            # hunt for the FF FF header
            b = self.ser.read(1)
            if not b:
                return None
            if prev == 0xFF and b[0] == 0xFF:
                break
            prev = b[0]
        head = self.ser.read(2)                # ID, LENGTH
        if len(head) < 2:
            return None
        servo_id, length = head[0], head[1]
        rest = self.ser.read(length)           # ERROR, PARAMS..., CHECKSUM
        if len(rest) < length:
            return None
        body = [servo_id, length, *rest[:-1]]
        if checksum(body) != rest[-1]:
            raise IOError(f"Bad checksum in reply from servo {servo_id}")
        raw = HEADER + bytes(head) + bytes(rest)
        return raw, servo_id, rest[0], bytes(rest[1:-1])

    def transact(self, servo_id, instruction, params=()):
        """Send one instruction packet and return the servo's status packet (or None)."""
        packet = build_packet(servo_id, instruction, params)
        self.ser.reset_input_buffer()
        self.ser.write(packet)
        self.ser.flush()
        for _ in range(2):                     # some adapters echo what we sent: skip it
            reply = self._read_packet()
            if reply is None:
                return None
            if reply[0] != packet:
                return reply
        return None

    # --- generic register access -------------------------------------------
    def read(self, servo_id, address, size):
        reply = self.transact(servo_id, INST_READ, [address, size])
        if reply is None:
            raise TimeoutError(f"No answer from servo {servo_id}")
        _, _, error, params = reply
        if error:
            print(f"Warning: servo {servo_id} reports error flags {error:#04x}")
        return params

    def write(self, servo_id, address, data):
        self.transact(servo_id, INST_WRITE, [address, *data])

    # SCS servos store 16-bit values BIG-endian (high byte first).
    def read_u16(self, servo_id, address):
        hi, lo = self.read(servo_id, address, 2)
        return (hi << 8) | lo

    def write_u16(self, servo_id, address, value):
        self.write(servo_id, address, [(value >> 8) & 0xFF, value & 0xFF])

    # --- friendly helpers -----------------------------------------------------
    def ping(self, servo_id):
        return self.transact(servo_id, INST_PING) is not None

    def torque(self, servo_id, on):
        self.write(servo_id, ADDR_TORQUE_ENABLE, [1 if on else 0])

    def read_position_deg(self, servo_id):
        return raw_to_degrees(self.read_u16(servo_id, ADDR_PRESENT_POSITION) & 0x3FF)

    def write_position_deg(self, servo_id, deg):
        self.write_u16(servo_id, ADDR_GOAL_POSITION, degrees_to_raw(deg))

    def write_speed_raw(self, servo_id, steps_per_second):
        self.write_u16(servo_id, ADDR_GOAL_SPEED, int(steps_per_second))

    def read_voltage(self, servo_id):
        return self.read(servo_id, ADDR_PRESENT_VOLTAGE, 1)[0] / 10

    def read_temperature(self, servo_id):
        return self.read(servo_id, ADDR_PRESENT_TEMPERATURE, 1)[0]


# ---------------------------------------------------------------------------
# 4. Try it
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    SERIAL_PORT = "COM11"   # <-- change me ("/dev/ttyACM0", "/dev/ttyUSB0", ...)

    # Sanity check of the packet builder (no hardware needed):
    # "write 818 (= +90 deg) to goal position of servo 1"
    pkt = build_packet(1, INST_WRITE, [ADDR_GOAL_POSITION, 0x03, 0x32])
    print("Example packet:", pkt.hex(" ").upper())   # FF FF 01 05 03 2A 03 32 97

    bus = ScsBus(SERIAL_PORT)
    try:
        for sid in range(1, 9):
            if bus.ping(sid):
                print(f"ID {sid}: pos={bus.read_position_deg(sid):+6.1f} deg  "
                      f"V={bus.read_voltage(sid):.1f} V  T={bus.read_temperature(sid)} C")
            else:
                print(f"ID {sid}: no answer")
            time.sleep(0.01)
    finally:
        bus.close()
```

### Block by block

* **Constants**: the instruction codes and register addresses from chapter 1.
* **`checksum` / `build_packet`**: implement the formulas above. `build_packet(1, INST_WRITE, [42, 0x03, 0x32])` returns the 9 bytes of the worked example.
* **`_read_packet`**: hunt for `FF FF`, read ID and LEN, then LEN bytes, verify the checksum.
* **`transact`**: flush old bytes, send, read the answer (skipping our own echo).
* **`read_u16` / `write_u16`**: 16-bit values are **big-endian** on SCS servos: `value = (hi << 8) | lo`.
* **`read_position_deg`**: `& 0x3FF` keeps the 10 position bits, then `(raw-511)/3.41`.

## 4.4 Run it

1. Put your port in `SERIAL_PORT`.
2. Power the bus (5 V) with all servos connected.
3. `python python/raw_protocol.py`

✅ **Checkpoint:**

```
Example packet: FF FF 01 05 03 2A 03 32 97
ID 1: pos=  -3.2 deg  V=5.0 V  T=29 C
ID 2: pos=  +1.8 deg  V=5.0 V  T=29 C
...
```

If you see `no answer` everywhere, don't debug this file first. Run `scan_bus.py` from
chapter 5, which uses the proven `rustypot` implementation.

## 4.5 From now on: rustypot

`rustypot` does exactly this, in Rust, with every register pre-declared and unit
conversions built in (radians, rad/s). Here is the mapping, so you know what each call sends:

| rustypot call | Packet |
|---|---|
| `c.ping(1)` | PING to ID 1 |
| `c.write_torque_enable(1, 1)` | WRITE addr 40, `[1]` |
| `c.write_goal_position(1, 1.5708)` | WRITE addr 42, raw(90°) = 818 big-endian |
| `c.write_goal_speed(1, 6)` | WRITE addr 46, 6 rad/s → 1173 steps/s |
| `c.read_present_position(1)` | READ addr 56, 2 bytes → radians, in a list |
| `c.sync_write_goal_position([1,2],[a,b])` | one SYNC_WRITE packet for both |

Next → [05 Scan the bus and set IDs](05_Scan_And_IDs.md)
