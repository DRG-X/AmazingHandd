# 03 · PC setup: Python, virtual environment, serial port

## 3.1 Install Python

* Python **3.10 to 3.13** works for the basic path (`rustypot` ships ready-made wheels for
  Windows, macOS, Linux x86-64 and Linux ARM64/Raspberry Pi).
* The advanced demo (chapters 10+) wants **Python 3.12**, so if you are installing
  fresh, install 3.12 now.

Windows: python.org installer, tick *"Add python.exe to PATH"*.
Ubuntu: `sudo apt install python3 python3-venv python3-pip`.
macOS: `brew install python@3.12`.

## 3.2 Create the project

```bash
mkdir my_amazing_hand
cd my_amazing_hand
mkdir python arduino
python -m venv .venv            # on Linux/macOS you may need: python3 -m venv .venv
```

Activate the virtual environment **every time** you open a new terminal:

```bash
# Windows (PowerShell)
.venv\Scripts\Activate.ps1
# Windows (cmd)
.venv\Scripts\activate.bat
# Linux / macOS
source .venv/bin/activate
```

Create `requirements.txt`:

```text
rustypot>=1.8
numpy
pyserial
```

* `rustypot`: servo driver used by all the repo's Python code
* `numpy`: `deg2rad`, `rad2deg`, `clip`…
* `pyserial`: only used by `raw_protocol.py` (chapter 4)

Install:

```bash
pip install -r requirements.txt
python -c "from rustypot import Scs0009PyController; print('rustypot OK')"
```

✅ **Checkpoint:** `rustypot OK` is printed.

## 3.3 Find the serial port of your adapter

Plug the adapter's USB cable (the 5 V supply can stay unplugged for this step).

**Windows**: Device Manager → *Ports (COM & LPT)* → e.g. `USB-SERIAL CH343 (COM11)`.
The repo examples use `"COM11"` / `"COM5"`, so yours will differ. If no port appears, install the
**CH343 (or CH340) driver** from the WCH website (the chip used on the Waveshare adapter).

**Linux**:

```bash
ls /dev/ttyACM* /dev/ttyUSB*      # typically /dev/ttyACM0 (the repo's default)
sudo dmesg | tail                 # shows what just got plugged in
sudo usermod -aG dialout $USER    # permission to open serial ports, then LOG OUT and back in
```

**macOS**:

```bash
ls /dev/tty.usb*                  # e.g. /dev/tty.usbmodem5A7A0185001
```

Python can list the ports for you too:

```bash
python -m serial.tools.list_ports -v
```

Write your port down. In every script of this tutorial there is a line

```python
SERIAL_PORT = "COM11"
```

which you replace with **your** port (`"/dev/ttyACM0"`, `"/dev/tty.usbmodem…"`…).

✅ **Checkpoint:** you know your port name.

Next → [04 The Feetech protocol](04_Servo_Protocol.md) (optional but recommended), or jump to
[05 Scan the bus](05_Scan_And_IDs.md).
