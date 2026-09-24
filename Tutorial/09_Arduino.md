# 09 · Path B: Arduino + TTLinker

Use this if you drive the hand from an Arduino instead of a PC. The concepts (IDs, middle
positions, mirrored motors, gestures) are the same as in chapters 1, 5 and 6; only the units
change: **raw steps** (0-1023, 511 = middle, 1 step = 0.293°) and speeds in **steps/s**.

## 9.1 Install

1. Arduino IDE 2.x.
2. *Library Manager* → search **SCServo** (by Feetech) → install (`assets/SCServo.jpg`).
3. Board: **Arduino Mega 2560** recommended. It has extra hardware serial ports, so the servo
   bus uses `Serial1` and the USB port stays free. An Uno/Nano works but shares its only
   serial port between USB and the bus.

## 9.2 Wiring

The **TTLinker mini** converts the Arduino's TX/RX pair into the servos' single half-duplex data wire.

* **Control mode** (`assets/Arduino_Control-mode.jpg`): Arduino TX → TTLinker RX,
  Arduino RX → TTLinker TX, GND ↔ GND, 5 V supply on the TTLinker's power input,
  servos on the TTLinker's bus connector.
  * Mega: pins **18 (TX1) / 19 (RX1)** → `Serial1`.
  * Uno/Nano: pins **1 (TX) / 0 (RX)** → `Serial`. **Unplug these two wires while uploading**,
    and you can't use the Serial Monitor, because the bus runs at 1 Mbaud on the same port.
* **Link mode** (`assets/Arduino_Link-mode.jpg`): used only to set IDs with the Feetech FD
  software *through* the Arduino's USB chip. Upload the empty *Blink* sketch first so the
  microcontroller leaves the serial lines alone, then wire as in the picture. The French tutorial
  linked in `ArduinoExample/README.md` walks through it.

## 9.3 Set the IDs

With FD in link mode (chapter 5.2 procedure): servos one at a time → IDs 1…8.

## 9.4 Sketch 1: `arduino/Amazing_Hand_Finger_Test/Amazing_Hand_Finger_Test.ino`

(Arduino wants each sketch in a folder of the same name.)

```cpp
// Amazing_Hand_Finger_Test.ino  (repo: ArduinoExample/Amazing_Hand-Finger_Test.ino)
// Closes, opens and wiggles ONE finger. Used to fine-tune MiddlePos_1 / MiddlePos_2.
// Positions are RAW servo units: 0..1023 over 300 deg, 511 = middle, 1 step = 0.293 deg.

#include <SCServo.h>

SCSCL sc;   // driver object for the SCS series (SCS0009 is an SCS servo)

// Finger parameters
int ID_1 = 1;           // change to the servo IDs you want to calibrate
int ID_2 = 2;
int MiddlePos_1 = 511;  // middle position of ID_1 (tune me)
int MiddlePos_2 = 511;  // middle position of ID_2 (tune me)

void setup()
{
  // Arduino MEGA: the servos are on Serial1 (pins 18 TX1 / 19 RX1).
  Serial1.begin(1000000);
  sc.pSerial = &Serial1;
  // Arduino UNO / NANO (one serial port only) use instead:
  //   Serial.begin(1000000);
  //   sc.pSerial = &Serial;
  delay(1000);
}

void loop()
{
  CloseFinger(); delay(3000);   // <- look at the servo horns now
  OpenFinger();  delay(500);
  Nonono();      delay(500);
}

void CloseFinger()
{
  // RegWritePos(ID, position, time, speed): stores the command in the servo...
  sc.RegWritePos(ID_1, MiddlePos_1 + 300, 0, 1500);  // +300 steps = +88 deg
  sc.RegWritePos(ID_2, MiddlePos_2 - 300, 0, 1500);  // -300 steps = -88 deg
  sc.RegWriteAction();                                // ...and this starts every stored command at once
}

void OpenFinger()
{
  sc.RegWritePos(ID_1, MiddlePos_1 - 100, 0, 1500);  // -29 deg
  sc.RegWritePos(ID_2, MiddlePos_2 + 100, 0, 1500);  // +29 deg
  sc.RegWriteAction();
}

void Nonono()
{
  // straight
  sc.RegWritePos(ID_1, MiddlePos_1 - 100, 0, 1500); sc.RegWritePos(ID_2, MiddlePos_2 + 100, 0, 1500); sc.RegWriteAction();
  for (int i = 0; i < 3; i++)
  {
    delay(300);
    // tilt one way (both motors shift in the same numeric direction)
    sc.RegWritePos(ID_1, MiddlePos_1, 0, 1200);       sc.RegWritePos(ID_2, MiddlePos_2 + 200, 0, 1200); sc.RegWriteAction();
    delay(300);
    // tilt the other way  (the original used MiddlePos_1 for ID_2 here: fixed)
    sc.RegWritePos(ID_1, MiddlePos_1 - 200, 0, 1200); sc.RegWritePos(ID_2, MiddlePos_2, 0, 1200);       sc.RegWriteAction();
  }
  // back to straight (same fix)
  sc.RegWritePos(ID_1, MiddlePos_1 - 100, 0, 1500); sc.RegWritePos(ID_2, MiddlePos_2 + 100, 0, 1500); sc.RegWriteAction();
  delay(400);
}
```

Key points:

* `SCSCL sc;` is the SCServo driver class for the **SCS** series (SMS/STS servos use other classes).
* `sc.pSerial = &Serial1;` tells the library which port the bus is on. `Serial1.begin(1000000)` sets 1 Mbaud.
* `RegWritePos(ID, Position, Time, Speed)` sends a **REG_WRITE**: the servo stores the goal but
  does not move yet. `Time = 0` means "use Speed" (steps/s).
* `RegWriteAction()` broadcasts **ACTION**: every servo that has a stored goal starts **at the
  same instant**. That is how the two motors of a finger stay synchronized (chapter 4.1).
* `+300` steps ≈ +88°, `-100` ≈ −29°. Same close/open logic as `finger_test.py`.
* **Fixed:** in the original `Nonono()`, two lines used `MiddlePos_1` for servo `ID_2`
  (copy/paste slip). It only matters once the two middles differ, but it would then tilt the finger wrong.

**Calibration** (guide pages 26-27): same as chapter 6.3, but values are raw: 511 = no
correction, `520` = +9 steps ≈ +2.6°. Record the 8 values.

## 9.5 Sketch 2: `arduino/Amazing_Hand_Demo/Amazing_Hand_Demo.ino`

```cpp
// Amazing_Hand_Demo.ino  (repo: ArduinoExample/Amazing_Hand_Demo.ino)
// Loops through all the gestures. Angles are given in DEGREES relative to the
// calibrated middle position and converted to raw steps in the Move_X functions.

#include <SCServo.h>

SCSCL sc;   // driver object for SCS series servos

// Side
int Side = 1; // replace "1" by "2" for left hand

//Speed 
int MaxSpeed =1500;
int CloseSpeed =750;

// Fingers middle poses (RAW units, 511 = theoretical middle), order:
//               ID1  ID2  ID3  ID4  ID5  ID6  ID7  ID8
int MiddlePos[8]={520,511,500,490,515,520,480,511}; // replace values by your calibration results


//Servo control
float Step = 0.293; // 300°/1024

void setup()
{
  // Arduino MEGA: servos on Serial1 (pins 18 TX1 / 19 RX1)
  Serial1.begin(1000000);
  sc.pSerial = &Serial1;
  // Arduino UNO / NANO: use Serial instead (and unplug RX/TX while uploading!)
  //   Serial.begin(1000000);
  //   sc.pSerial = &Serial;
  delay(1000);
}



void loop()
{
  
  OpenHand();delay(500);

  CloseHand();delay(3000);
  
  OpenHand_Progressive();delay(400);

  SpreadHand();delay(600);
  ClenchHand();delay(600);

  OpenHand();delay(200);

  Index_Pointing();delay(400);
  Nonono();delay(500);
 
  OpenHand();delay(300);

  Perfect();delay(800);

  OpenHand();delay(400);

  Victory();delay(1000);
  Scissors();delay(500);

  OpenHand();delay(400);

  Pinched(); delay(1000);

  MiddleFinger();delay(800);

  

  
}



void OpenHand()
{
  Move_Index (-35, 35, MaxSpeed);
  Move_Middle (-35, 35, MaxSpeed);
  Move_Ring (-35, 35, MaxSpeed);
  Move_Thumb (-35, 35, MaxSpeed);

}

void CloseHand()
{
  Move_Index (90, -90, CloseSpeed);
  Move_Middle (90, -90, CloseSpeed);
  Move_Ring (90, -90, CloseSpeed);
  Move_Thumb (70, -70, CloseSpeed+200);
  
}

void OpenHand_Progressive()
{
  Move_Index (-35, 35, 1200); // Open Index
  delay(300);
  Move_Middle (-35, 35, 1200); // Open Middle
  delay(300);
  Move_Ring (-35, 35, 1200); // Open Ring
  delay(300);
  Move_Thumb (-35, 35, MaxSpeed); // Open Thumb 
}

void SpreadHand()
{
  if (Side==1) // Right Hand
  {
    Move_Index (4, 90, MaxSpeed);
    Move_Middle (-32, 32, MaxSpeed);
    Move_Ring (-90, -4, MaxSpeed);
    Move_Thumb (-90, -4, MaxSpeed);  
  } 
  else if (Side==2) // Left Hand
  {
    Move_Index (-60, 0, MaxSpeed);
    Move_Middle (-35, 35, MaxSpeed);
    Move_Ring (-4, 90, MaxSpeed);
    Move_Thumb (-4, 90, MaxSpeed);  
  }
}

void ClenchHand()
{
  if (Side==1) // Right Hand
  {
    Move_Index (-60, 0, MaxSpeed);
    Move_Middle (-35, 35, MaxSpeed);
    Move_Ring (0, 70, MaxSpeed);
    Move_Thumb (-4, 90, MaxSpeed);  
  }


  else if (Side==2) // Left Hand
  {
    Move_Index (0, 60, MaxSpeed);
    Move_Middle (-35, 35, MaxSpeed);
    Move_Ring (-70, 0, MaxSpeed);
    Move_Thumb (-90, -4, MaxSpeed);
  }
  
}



void Index_Pointing()
{
  Move_Index (-40, 40, MaxSpeed);
  Move_Middle (90, -90, MaxSpeed);
  Move_Ring (90, -90, MaxSpeed);
  Move_Thumb (90, -90, MaxSpeed);
}

void Nonono()
{
  Index_Pointing();
  for (int i=0;i<3;i++)
  {
    delay(300);
    Move_Index (-10, 80, MaxSpeed);
    delay(300);
    Move_Index (-80, 10, MaxSpeed);
  }
  Move_Index (-35, 35, MaxSpeed);
  delay(400);
}

void Perfect()
{
  if (Side==1) //Right Hand
  {
    Move_Index (50, -50, MaxSpeed);
    Move_Middle (0, -0, MaxSpeed);
    Move_Ring (-20, 20, MaxSpeed);
    Move_Thumb (65, 12, MaxSpeed);

  }
  else if (Side==2) // Left Hand
  {
    Move_Index (50, -50, MaxSpeed);
    Move_Middle (0, -0, MaxSpeed);
    Move_Ring (-20, 20, MaxSpeed);
    Move_Thumb (-12, -65, MaxSpeed);
  }
}

void Victory()
{
  if (Side==1) //Right Hand
  {
    Move_Index (-15, 65, MaxSpeed);
    Move_Middle (-65, 15, MaxSpeed);
    Move_Ring (90, -90, MaxSpeed);
    Move_Thumb (90, -90, MaxSpeed);

  }
  else if (Side==2) // Left Hand
  {
    Move_Index (-65, 15, MaxSpeed);
    Move_Middle (-15, 65, MaxSpeed);
    Move_Ring (90, -90, MaxSpeed);
    Move_Thumb (90, -90, MaxSpeed);
  }
}

void Pinched()
{
  if (Side==1) //Right Hand
  {
    Move_Index (90, -90, MaxSpeed);
    Move_Middle (90, -90, MaxSpeed);
    Move_Ring (90, -90, MaxSpeed);
    Move_Thumb (0, -75, MaxSpeed);

  }
  else if (Side==2) // Left Hand
  {
    Move_Index (90, -90, MaxSpeed);
    Move_Middle (90, -90, MaxSpeed);
    Move_Ring (90, -90, MaxSpeed);
    Move_Thumb (75, 0, MaxSpeed);
  }
}

void Scissors()
{
  Victory();
  if (Side==1) //Right
  {
    for (int i=0;i<3;i++)
    {
      delay(300);
      Move_Index (-50, 20, MaxSpeed);
      Move_Middle (-20, 50, MaxSpeed);
      
      delay(300);
      Move_Index (-15, 65, MaxSpeed);
      Move_Middle (-65, 15, MaxSpeed);
    }  

  }
  else if (Side==2) // Left Hand
  {
    for (int i=0;i<3;i++)
    {
      delay(300);
      Move_Index (-20, 50, MaxSpeed);
      Move_Middle (-50, 20, MaxSpeed);
      
      delay(300);
      Move_Index (-65, 15, MaxSpeed);
      Move_Middle (-15, 65, MaxSpeed);
    }
  }
}


void MiddleFinger()  // "Fuck()" in the original
{
  if (Side==1) //Right Hand
  {
    Move_Index (90, -90, MaxSpeed);
    Move_Middle (-35, 35, MaxSpeed);
    Move_Ring (90, -90, MaxSpeed);
    Move_Thumb (0, -75, MaxSpeed);

  }
  else if (Side==2) // Left Hand
  {
    Move_Index (90, -90, MaxSpeed);
    Move_Middle (-35, 35, MaxSpeed);
    Move_Ring (90, -90, MaxSpeed);
    Move_Thumb (75, 5, MaxSpeed);
  }
}

// Move_X(angle motor1 [deg], angle motor2 [deg], speed [steps/s])
// raw position = middle + angle / 0.293   (e.g. +90 deg -> +307 steps)
void Move_Index (float Pos_1, float Pos_2, int Speed)
{
  sc.RegWritePos(1, MiddlePos[0]+Pos_1/Step, 0, Speed);sc.RegWritePos(2, MiddlePos[1]+Pos_2/Step, 0, Speed);sc.RegWriteAction();
}
void Move_Middle (float Pos_1, float Pos_2, int Speed)
{
  sc.RegWritePos(3, MiddlePos[2]+Pos_1/Step, 0, Speed);sc.RegWritePos(4, MiddlePos[3]+Pos_2/Step, 0, Speed);sc.RegWriteAction();
}
void Move_Ring (float Pos_1, float Pos_2, int Speed)
{
  sc.RegWritePos(5, MiddlePos[4]+Pos_1/Step, 0, Speed);sc.RegWritePos(6, MiddlePos[5]+Pos_2/Step, 0, Speed);sc.RegWriteAction();
}
void Move_Thumb (float Pos_1, float Pos_2, int Speed)
{
  sc.RegWritePos(7, MiddlePos[6]+Pos_1/Step, 0, Speed);sc.RegWritePos(8, MiddlePos[7]+Pos_2/Step, 0, Speed);sc.RegWriteAction();
}
  
```

How it maps to the Python demo:

| Python (`hand_demo.py`) | Arduino |
|---|---|
| `MiddlePos` in degrees | `MiddlePos[8]` in raw steps |
| `MaxSpeed = 7` rad/s | `MaxSpeed = 1500` steps/s (≈ 7.7 rad/s) |
| `np.deg2rad(MiddlePos[k] + angle)` | `MiddlePos[k] + angle / Step` with `Step = 0.293` |
| two `write_goal_position` calls | two `RegWritePos` + one `RegWriteAction` |
| `time.sleep(s)` | `delay(ms)` |

Gestures and angles are the same as in Python (they are in degrees in both). Only the
conversion in `Move_X` differs. The thumb closes with `(70, -70)` here instead of `(90, -90)`.

## 9.6 Converting calibrations between Python and Arduino

```
arduino_raw = 511 + python_deg / 0.293          python_deg = (arduino_raw - 511) * 0.293
```

Example: the Arduino demo's `{520,511,500,490,515,520,480,511}` ≈ Python
`[2.6, 0, -3.2, -6.2, 1.2, 2.6, -9.1, 0]`.

✅ **Checkpoint:** after uploading `Amazing_Hand_Demo.ino` with your values (and the right
`Serial`/`Serial1` choice), the hand plays the same choreography as the Python demo.

Next → [10 Advanced demo](10_Advanced_Setup.md) (needs the PC-side USB adapter)
