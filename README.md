LIDAR Lite v3 ENHANCED
=========================

Introduction
--------------
This library tends to improve the efficiency of the acquisition of LidarLite v2 (deprecated) and LidarLite v3 (release from Garmin) lasermeter.

Improvements over the original library
--------------------
- *Asynchronous acquisition*
- Non-blocking architecture
- *Automatic reset* of the lidars
- *State machine* for each lidar
- *Maximum speed* readings for many lidars (enable 400kHz I2C)
- The whole library with Serial & Arduino firmware weight *7200 bytes* of the memory for 
 lasers

Limitations
-----------
- Software limitation to 8 lasers, 8 lasers uses as much memory as for 1 laser
- Cannot predict when you got a new data but a callback is easy to implement.
- There is no velocity reading (nor scale configuration)

Usage 
-------

```c++
#include "LidarObject.h"
#include "LidarController.h"
#include "I2CFunctions.h"

#include <Wire.h>
#define WIRE400K false


#define Z1_LASER_TRIG 11
#define Z1_LASER_EN 12
#define Z1_LASER_PIN 13
#define Z1_LASER_AD 0x6E

#define DATARATE 100 
// 100Hz
#define DELAY_SEND_MICROS 1000000/DATARATE


static LidarController Controller;
static LidarObject LZ1;


// Delays
long now, last;

void initLidars() {
  // Initialisation of the lidars objects
  LZ1.begin(Z1_LASER_EN, Z1_LASER_PIN, Z1_LASER_AD, 2, 'a');
  
  // Initialisation of the controller
  Controller.begin(WIRE400K);
  delay(100);
  Controller.add(&LZ1, 0);
}

void setup() {
  Serial.begin(57600);
  while (!Serial); // for compatibility
  initLidars();
  last = micros();
}

void loop() {
  Controller.spinOnce();
  now = micros();
  if(now - last > DELAY_SEND_MICROS){
    last = micros();
    laserprint();
  } 
}

void laserprint(){
  Serial.print(" Measure: ");
  Serial.print(Controller.distances[0]);
  Serial.print(" Status: ");
  Serial.println(Controller.statuses[0]);
  Serial.print(" Signal Strenght: ");
  Serial.println(Controller.signal_strenghts[0]);
}
```

Code
-------

### Lidar object
The lidar object represents a laser.

#### LidarObject::begin
```C++
void LidarObject.begin(EnablePin, ModePin, Address, LaserConfiguration, OneCharName);
LZ1.begin(12, 13, 0x64, 2, 'x');
```

#### LIDAR_STATE enum

```C++
enum LIDAR_STATE {
  SHUTING_DOWN = 240,       // Shutdown the laser to reset it
  NEED_RESET = 48,          // Too much outliers, need to reset
  RESET_PENDING = 80,       // Wait 15ms after you reset the Lidar, we are waiting in this state
  NEED_CONFIGURE = 144,     // 15ms passed, we now configure the Lidar
  ACQUISITION_READY = 32,   // I started an acquisition, need someone to read it
  ACQUISITION_PENDING = 64, // The acquisition in on progress
  ACQUISITION_DONE = 128    // I read the data, need to start an acq again
};
```

### LidarController object

#### LidarController::begin
Start the I2C line with or without fasti2c (400kHz)
```C++
void begin(bool fasti2c = false);
```

#### LidarController::add
Add a lidar to the controller, if the id is over the maximum number of lidar (8 by default), returns false and do not add the lidar.

```C++
bool add(LidarObject* _Lidar, byte _id);
```

#### LidarController::spinOnce

Make a new step in the Lidar State Machine, store results to *int LidarController::distances*

The functions below are NOT needed if you do not want to use the state machine

```C++
void spinOnce();
```

#### LidarController::configure

Send the Lidar configuration to the Lidar (id of the lidar)
  - 0 = basic configuration
  - 1 = faster (Do not read 3 times), bit noisier
  - 2 = low noise, low sensitivity, less false detection (Default)
  - 3 = High noise, high sensitivity

```C++
configure(byte Lidar = 0, byte configuration = 2);
```

#### LidarController::changeAddress

Change the address of the Lidar to the address configured in the object
```C++
changeAddress(byte Lidar);
```

#### LidarController::status

Returns the status byte of the Lidar (id)

```C++
byte status(byte Lidar = 0);
```
#### LidarController::async

Send the command to the lidar (id) to start an acquisition

```C++
byte async(byte Lidar = 0);
```

#### LidarController::distance

Read the distance from the Lidar (id)

NOTE : You have to send the command async FIRST and should check if the data is there. You can know it by reading the bit 0 from the status of the lidar (LidarController::status)

```C++
int distance(byte Lidar, int * data);
```

#### LidarController::distanceAndAsync

Read the distance from the Lidar (id) and restart an acquisition, writes to data and returns nack status

```C++
byte distanceAndAsync(byte Lidar, int * data);
```

#### LidarController::resetLidar

Reset a Lidar

```C++
void resetLidar(byte Lidar = 0)
```


### I2C object

#### I2C::begin
Start the I2C line
```C++
void I2C.begin(fasti2c = false);
```

#### I2C::isOnline
Check if the device/laser is online 
```C++
void I2C.isOnline(Device = 0x62);
```

#### I2C::write
Write data though I2C to a device
```C++
byte write(byte Device, byte regAdr, byte data);
```

#### I2C::readByte 
Read one byte from an I2C device, stored in data, returns NACK status
```C++
byte readByte(byte Device, byte regAdr, byte * data)
```

#### I2C::readWord
Read *two* byte from an I2C device, stored in data, returns NACK status
```C++
byte readWord(byte Device, byte regAdr, byte * data)
```


### Full example

```C++
#include "LidarObject.h"
#include "LidarController.h"
#include "I2CFunctions.h"

#include <Wire.h>
#define WIRE400K false
/*** Defines : CONFIGURATION ***/
// Defines Trigger
#define Z1_LASER_TRIG 11
#define Z2_LASER_TRIG 8
#define Z3_LASER_TRIG 5
#define Z4_LASER_TRIG 2
#define Z5_LASER_TRIG 16
#define Z6_LASER_TRIG 19
// Defines power enable lines of laser
#define Z1_LASER_EN 12
#define Z2_LASER_EN 9
#define Z3_LASER_EN 6
#define Z4_LASER_EN 3
#define Z5_LASER_EN 15
#define Z6_LASER_EN 18
// Defines laser mode 
#define Z1_LASER_PIN 13
#define Z2_LASER_PIN 10
#define Z3_LASER_PIN 7
#define Z4_LASER_PIN 4
#define Z5_LASER_PIN 14
#define Z6_LASER_PIN 17
//Define address of lasers
//Thoses are written during initialisation
// default address : 0x62
#define Z1_LASER_AD 0x6E
#define Z2_LASER_AD 0x66
#define Z3_LASER_AD 0x68
#define Z4_LASER_AD 0x6A
#define Z5_LASER_AD 0x6C
#define Z6_LASER_AD 0x64

#define NUMBER_OF_LASERS 6

// Maximum datarate
#define DATARATE 100
// Actual wait between communications 100Hz = 10ms
#define DELAY_SEND_MICROS 1000000/DATARATE

// Lidars
static LidarController Controller;
static LidarObject LZ1;
static LidarObject LZ2;
static LidarObject LZ3;
static LidarObject LZ4;
static LidarObject LZ5;
static LidarObject LZ6;

// Delays
long now, last;

void beginLidars() {
  // Initialisation of the lidars objects
  LZ1.begin(Z1_LASER_EN, Z1_LASER_PIN, Z1_LASER_AD, 2, 'x');
  LZ2.begin(Z2_LASER_EN, Z2_LASER_PIN, Z2_LASER_AD, 2, 'X');
  LZ3.begin(Z3_LASER_EN, Z3_LASER_PIN, Z3_LASER_AD, 2, 'y');
  LZ4.begin(Z4_LASER_EN, Z4_LASER_PIN, Z4_LASER_AD, 2, 'Z');
  LZ5.begin(Z5_LASER_EN, Z5_LASER_PIN, Z5_LASER_AD, 2, 'y');
  LZ6.begin(Z6_LASER_EN, Z6_LASER_PIN, Z6_LASER_AD, 2, 'Z');
  
  // Initialisation of the controller
  Controller.begin(WIRE400K);
  delay(100);
  Controller.add(&LZ1, 0);
  Controller.add(&LZ2, 1);
  Controller.add(&LZ3, 2);
  Controller.add(&LZ4, 3);
  Controller.add(&LZ5, 4);
  Controller.add(&LZ6, 5);
}

void setup() {
  Serial.begin(57600);
  while (!Serial);
  beginLidars();
  last = micros();
}

void loop() {
  Controller.spinOnce();
  now = micros();
  if(now - last > DELAY_SEND_MICROS){
    last = micros();
    laserprint();
  } 
}

void laserprint(){
  for(byte i = 0; i < 6; i++){
    Serial.print(i);
    Serial.print(" - ");
    Serial.print(Controller.distances[i]);
    Serial.print(" - ");
    Serial.println(Controller.statuses[i]);
  }
}
```

### Explanations

#### Include the libraries and add some defines to use later

```C++
#include "LidarObject.h"
#include "LidarController.h"
#include "I2CFunctions.h"

#include <Wire.h>
#define WIRE400K false
/*** Defines : CONFIGURATION ***/
// Defines Trigger
#define Z1_LASER_TRIG 11
#define Z2_LASER_TRIG 8
#define Z3_LASER_TRIG 5
#define Z4_LASER_TRIG 2
#define Z5_LASER_TRIG 16
#define Z6_LASER_TRIG 19
// Defines power enable lines of laser
#define Z1_LASER_EN 12
#define Z2_LASER_EN 9
#define Z3_LASER_EN 6
#define Z4_LASER_EN 3
#define Z5_LASER_EN 15
#define Z6_LASER_EN 18
// Defines laser mode 
#define Z1_LASER_PIN 13
#define Z2_LASER_PIN 10
#define Z3_LASER_PIN 7
#define Z4_LASER_PIN 4
#define Z5_LASER_PIN 14
#define Z6_LASER_PIN 17
//Define address of lasers
//Thoses are written during initialisation
// default address : 0x62
#define Z1_LASER_AD 0x6E
#define Z2_LASER_AD 0x66
#define Z3_LASER_AD 0x68
#define Z4_LASER_AD 0x6A
#define Z5_LASER_AD 0x6C
#define Z6_LASER_AD 0x64

#define NUMBER_OF_LASERS 6

// Maximum datarate
#define DATARATE 100
// Actual wait between communications 100Hz = 10ms
#define DELAY_SEND_MICROS 1000000/DATARATE

// Lidars
static LidarController Controller;
static LidarObject LZ1;
static LidarObject LZ2;
static LidarObject LZ3;
static LidarObject LZ4;
static LidarObject LZ5;
static LidarObject LZ6;

// Delays
long now, last;
```

#### Initiation

Initiate Laser objects, begin the I2C and finally add lidars to the controller 

```C++

// Initialisation of the lidars objects
  LZ1.begin(Z1_LASER_EN, Z1_LASER_PIN, Z1_LASER_AD, 2, 'x');
  LZ2.begin(Z2_LASER_EN, Z2_LASER_PIN, Z2_LASER_AD, 2, 'X');
  LZ3.begin(Z3_LASER_EN, Z3_LASER_PIN, Z3_LASER_AD, 2, 'y');
  LZ4.begin(Z4_LASER_EN, Z4_LASER_PIN, Z4_LASER_AD, 2, 'Z');
  LZ5.begin(Z5_LASER_EN, Z5_LASER_PIN, Z5_LASER_AD, 2, 'y');
  LZ6.begin(Z6_LASER_EN, Z6_LASER_PIN, Z6_LASER_AD, 2, 'Z');
  
  // Initialisation of the controller
  Controller.begin(WIRE400K);
  delay(100);
  Controller.add(&LZ1, 0);
  Controller.add(&LZ2, 1);
  Controller.add(&LZ3, 2);
  Controller.add(&LZ4, 3);
  Controller.add(&LZ5, 4);
  Controller.add(&LZ6, 5);
```

#### Update the state machine

You just have to use Controller.spinOnce() and it updates the state machine, send/retrieves data to/from lidars.

Then, every DELAY_SEND_MICROS microseconds, get the data

```C++
void loop(){
  Controller.spinOnce();
  now = micros();
  if(now - last > DELAY_SEND_MICROS){
    last = micros();
    laserprint();
} 
```

#### Data extraction

```C++ 
void laserprint(){
  for(byte i = 0; i < 6; i++){
    Serial.print(i);
    Serial.print(" - ");
    Serial.print(Controller.distances[i]);
    Serial.print(" - ");
    Serial.println(Controller.statuses[i]);
  }
}
```


Todo
--------
- Library configurations to avoid to open the library to use our personnal parameters
- Velocity readings
- Independant callbacks for each lasers or when each are done, or one single callback for every lasers

Pull requests 
----------
Pull requests are welcome :D

Credits 
------
Alexis Paques (alexis[dot]paques[at]gmail[dot]com)