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
  LZ1.begin(Z1_LASER_EN, Z1_LASER_PIN, Z1_LASER_TRIG, Z1_LASER_AD, 2, DISTANCE, 'x');
  LZ2.begin(Z2_LASER_EN, Z2_LASER_PIN, Z2_LASER_TRIG, Z2_LASER_AD, 2, DISTANCE, 'X');
  LZ3.begin(Z3_LASER_EN, Z3_LASER_PIN, Z3_LASER_TRIG, Z3_LASER_AD, 2, DISTANCE, 'y');
  LZ4.begin(Z4_LASER_EN, Z4_LASER_PIN, Z4_LASER_TRIG, Z4_LASER_AD, 2, DISTANCE, 'Z');
  LZ5.begin(Z5_LASER_EN, Z5_LASER_PIN, Z5_LASER_TRIG, Z5_LASER_AD, 2, DISTANCE, 'y');
  LZ6.begin(Z6_LASER_EN, Z6_LASER_PIN, Z6_LASER_TRIG, Z6_LASER_AD, 2, DISTANCE, 'Z');
  
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
// UNSAFE, as you could read out of the array.
// You can also use LZ1.distance
  for(byte i = 0; i < NUMBER_OF_LASERS; i++){
    Serial.print(i);
    Serial.print("\t");
    Serial.print(Controller.lidars[i]->distance);
    Serial.print("\t");
    Serial.print(Controller.lidars[i]->velocity);
    Serial.print("\t");
    Serial.print(Controller.lidars[i]->strength);
    Serial.print("\t");
  }
  Serial.println("");
}
