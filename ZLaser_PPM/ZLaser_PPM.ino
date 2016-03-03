#include "LidarObject.h"
#include "LidarController.h"
#include "I2CFunctions.h"

#include <Wire.h>
#define WIRE400K false
/*** Defines : CONFIGURATION ***/
// Defines laser ready data
#define Z1_LASER_PIN 10
#define Z2_LASER_PIN 8
// Defines power enable lines of laser
#define Z1_LASER_EN 11
#define Z2_LASER_EN 9
//Define address of lasers
//Thoses are written during initialisation
// default address : 0x62
#define Z1_LASER_AD 0x64
#define Z2_LASER_AD 0x66

#define NUMBER_OF_LASERS 2

#define READINESS true

// Lidars
static LidarController Controller;
static LidarObject LZ1;
static LidarObject LZ2;

void beginLidars() {
  // Initialisation of the lidars objects
  LZ1.begin(Z1_LASER_EN, Z1_LASER_PIN, Z1_LASER_AD, 2, 'z');
  LZ2.begin(Z2_LASER_EN, Z2_LASER_PIN, Z2_LASER_AD, 2, 'Z');
  // Initialisation of the controller
  Controller.begin(WIRE400K);
  delay(10);
  Controller.add(&LZ1, 0);
  Controller.add(&LZ2, 1);
}

void setup() {
  Serial.begin(230400);
  while (!Serial);
  beginLidars();
  I2C.scan();
}

void loop() {
  Controller.spinOnce();
}
