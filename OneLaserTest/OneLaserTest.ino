#include "D:\Workspace\Github\LIDARLite_v2_Enchanced\LidarObject.h"
#include "D:\Workspace\Github\LIDARLite_v2_Enchanced\LidarController.h"
#include "D:\Workspace\Github\LIDARLite_v2_Enchanced\I2CFunctions.h"

#include <Wire.h>
#define WIRE400K true
/*** Defines : CONFIGURATION ***/
// Defines laser ready data
#define Z1_LASER_PIN 3
// Defines power enable lines of laser
#define Z1_LASER_EN 11
//Define address of lasers
//Thoses are written during initialisation
// default address : 0x62
#define Z1_LASER_AD 0x62

#define NUMBER_OF_LASERS 1


// Lidars
static LidarController Controller;
static LidarObject LZ1;

void beginLidars(){
  // Initialisation of the lidars objects
  LZ1.begin(Z1_LASER_EN, Z1_LASER_PIN, Z1_LASER_AD, 2, 'z');
  // Initialisation of the controller
  Controller.begin(WIRE400K);

  // Adding Lidars to the controller
  Controller.add(&LZ1, 0);

  delay(15);
 // Controller.changeAllAddresses();
 // delay(10);
}

bool isPerfect(int data){
  if(data > 1000){
    return false;
  }
  if(data < 15){
    return false;
  }
  return true;
}

void setup(){
  Serial.begin(57600);
  while(!Serial);
  beginLidars();
  Controller.asyncAll();
}

void loop(){
  outputASCII();
  delay(4); // 4 ms = 250Hz
  test();
}

void test(){
  Serial.println("for(byte i = 5; i > 0; --i) :");
  for(byte i = 5; i > 0; --i){
    Serial.print(i);
  }
  Serial.println("byte i = 5; i >= 0; i--:");
  for(int8_t i = 5; i >= 0; i--){
    Serial.print(i);
  }
}

void outputASCII(){
  // output as : 
  // int,int,int,int,int,int\n
  int data = 0;
  byte status = 10;
  Controller.async(0);
  long start = micros();
  status = Controller.status(0);
  while(bitRead(status,0) != 0){
    //Serial.print("status :");
    //Serial.println(status);
    //delayMicroseconds(0);
    status = Controller.status(0);
  }
  Controller.distance(0, &data);
  long stop = micros();
  Serial.print("time :");
  Serial.println(stop-start);
  
  Serial.print("data :");
  Serial.println(data);
  Serial.println("\n");
}

