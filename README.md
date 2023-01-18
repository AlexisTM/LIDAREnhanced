LIDARLite v3 ENHANCED
=========================

> Due to the lack of hardware, this goes unmaintained.

Introduction
--------------
This library tends to improve the **efficiency** and the **robustness** of the poor original acquisition library. It is originally released for the LidarLite v2 (deprecated) and compatible with the LidarLite v3 (release from Garmin) lasermeter. 

Improvements over the original library
--------------------
- *Asynchronous acquisition*
- **Non-blocking** architecture
- *Automatic reset* of the lidars if too much errors
- *State machine* for each lidar
- *Maximum speed* readings for many lidars (enable 400kHz I2C)
- Don't think about lasers anytime, it just make the acquisition in the background.


> It is important to note this was developed for a thesis and was not maintained afterwards as I do not have the hardware anymore. Feel free to refactor it, and improve code quality. If you want to maintain the library, I would happily give you ownership.
> At the time (2016), we needed much faster measurements than the provided library was capable of.

Limitations
-----------
- You have to edit the `MAX_LIDARS` in `LidarController.h` to set the number of instances of lidars you will use (default : 8)
- There is not (yet) velocity reading, it will be implemented with software (to avoid the badly designed blocking architecture) 
- Speed limited by the I2C bandwidth

How fast is it ?
-----------

The maximal speed is mainly limited by I2C speed more than the configuration. Therefore, I advise you to choose the most *stable* than the most speedy. The next step to improve performances is to sync the laser with the expected acquisition time depending on the distance precedently measured. It would allow to improve acquisition and reduce the I2C usage. Also, it could be great to use the constant acquisition.

* On 100 kHz i2c bus
  - 800Hz With FORCE_RESET_OFFSET **and** ENABLE_STRENGTH_MEASURE to **true**
  - 1050Hz With FORCE_RESET_OFFSET **or** ENABLE_STRENGTH_MEASURE to **true**
  - 1300Hz With FORCE_RESET_OFFSET **and** ENABLE_STRENGTH_MEASURE to **false**
* On 400 kHz i2c bus
  - 1325Hz With FORCE_RESET_OFFSET **and** ENABLE_STRENGTH_MEASURE to **true**
  - 1575Hz With FORCE_RESET_OFFSET **or** ENABLE_STRENGTH_MEASURE to **true**
  - 2080Hz With FORCE_RESET_OFFSET **and** ENABLE_STRENGTH_MEASURE to **false**


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

// 100Hz
#define DATARATE 100 
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
  Serial.print(LZ1.distance);
  Serial.print(" Signal strength: ");
  Serial.println(LZ1.strength);
  Serial.print(" Velocity: ");
  Serial.println(LZ1.velocity);
}
```

API
-------

Here is the API basis, you can check the [In-depth API](API_depth.md) to contribute.

### Lidar object
The lidar object represents a laser.

#### LidarObject::begin

```C++
void LidarObject.begin(EnablePin, ModePin, I2CAddress, LaserConfiguration, LidarMode, OneCharName);

// used as 
static LidarObject LZ1;
LZ1.begin(12, 13, 0x64, 2, DISTANCE, 'x');
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

#### LIDAR_MODE enum

```C++
enum LIDAR_MODE {
  NONE = 0,                  // Should not be used unless you want to disable a connected laser (could be implemented)
  DISTANCE = 1,              // Measure distance as fast as possible
  VELOCITY = 2,              // Measure velocity at a certain rate
  DISTANCE_AND_VELOCITY = 3  // Measure distance and velocity
};
```

#### Data reading

```C++ 
    int distance;      // newest measure
    int last_distance; // last measure
    int velocity;       // newest velocity
    uint8_t strength;   // newest signal strength
```

#### Callback configuration 

Check the [full example](example/Callback)) !

```C++ 

// Define the callback
void distance_callback(LidarObject* self){
  // self is the laser transmitting the interrupt. 
  Serial.print(self->name);
  Serial.print(":");
  Serial.println(self->distance);
}

// In the setup, link the callback :
LZ1.setCallbackDistance(&distance_callback);
LZ2.setCallbackDistance(&distance_callback);
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
bool add(LidarObject* _Lidar, uint8_t _id);
```

#### LidarController::spinOnce

Make a new step in the Lidar State Machine, store results to *int LidarController::distances*

The functions below are NOT needed if you do not want to use the state machine

```C++
void spinOnce();
```

#### LidarController::lasers

The lidars object is an array with the pointers to all the lasers. 

```C++
LidarObjects lidars[MAX_LIDARS];

// Can be used as 
Serial.print(Controller.lidars[0]->distance);
```

### Six lasers example

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
//Those are written during initialisation
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
  for(uint8_t i = 0; i < 6; i++){
    Serial.print(i);
    Serial.print(" - ");
    Serial.print(Controller.lidars[i]->distance);
    Serial.print(" - ");
    Serial.println(Controller.lidars[i]->strength);
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
//Those are written during initialisation
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
  for(uint8_t i = 0; i < 6; i++){
    Serial.print(i);
    Serial.print(" - ");
    Serial.print(Controller.lidars[i]->distance);
    Serial.print(" - ");
    Serial.println(Controller.lidars[i]->strength);
  }
}
```


Todo
--------
- Velocity readings
- Independent callbacks for each lasers or when each are done, or one single callback for every lasers

Pull requests 
----------
Pull requests are welcome :D

Credits 
------
* Alexis Paques (alexis[dot]paques[at]gmail[dot]com)
