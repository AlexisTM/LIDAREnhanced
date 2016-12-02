#include <Arduino.h>
#include <Wire.h>
#include <I2CFunctions.h> 
#include <LidarObject.h>
#include <LidarController.h>

#define WIRE400K true
// Trigger pin, can be unplugged
#define Z1_LASER_TRIG 11
// Enable pin, IMPORTANT
#define Z1_LASER_EN 12
// Mode pin, can be unplugged
#define Z1_LASER_PIN 13
//Define address of lasers
//Thoses are written during initialisation
// default address : 0x62
#define Z1_LASER_AD 0x6E

#define NUMBER_OF_LASERS 1

// Create lasers
static LidarController Controller;
static LidarObject LZ1;

void setup()
{
  Serial.begin(115200);
  // Configure lasers
  LZ1.begin(Z1_LASER_EN, Z1_LASER_PIN, Z1_LASER_TRIG, Z1_LASER_AD, 2, DISTANCE, 'A');
  LZ1.setCallbackDistance(&distance_callback);
  // Add the laser to the Controller
  Controller.add(&LZ1, 0);

  delay(100);
  Controller.begin(WIRE400K);
  delay(100);
}

void distance_callback(LidarObject* self){
   Serial.println(self->distance);
}

void loop()
{
  Controller.spinOnce();
  // Rest of your non blocking application. 
}
