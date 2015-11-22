#include "LidarEnchanced.h"
#include <Wire.h>


/*** Defines : CONFIGURATION ***/
// Defines laser ready data
#define X_LASER_PIN 3
#define Y_LASER_PIN 5
#define Z_LASER_PIN 7
#define W_LASER_PIN 9
// Defines power enable lines of laser
#define X_LASER_EN 2
#define Y_LASER_EN 4
#define Z_LASER_EN 6
#define W_LASER_EN 8
//Define address of lasers
//Thoses are written during initialisation
// default address : 0x62
#define X_LASER_AD 0x64
#define Y_LASER_AD 0x66
#define Z_LASER_AD 0x68
#define W_LASER_AD 0x6A

//Concatenate enable pins into a variable
char names[] = {'X', 'Y', 'Z', 'W'};
// Array of pins connected to the sensor Power Enable lines
byte sensorPinsEN[] = {X_LASER_EN, Y_LASER_EN, Z_LASER_EN, W_LASER_EN};
//Concatenate adresses into a variable
byte addresses[] = {X_LASER_AD, Y_LASER_AD, Z_LASER_AD, W_LASER_AD};

#define READINESS true

LIDAREnchanced L;

int oneShot(byte Lidar = 0x62){
  L.async(Lidar);
  while(L.isBusy(Lidar)){}
  return L.distance(Lidar);
}

void shotThemAll(){
  L.async(0x62);
  L.async(0x64);
  L.async(0x66);
  L.async(0x68);
  L.async(0x6A);
  Serial.print("data : ");
  Serial.print(L.distance(0x62));
  Serial.print("\t");
  Serial.print(L.distance(0x64));
  Serial.print("\t");
  Serial.print(L.distance(0x66));
  Serial.print("\t");
  Serial.print(L.distance(0x68));
  Serial.print("\t");
  Serial.println(L.distance(0x6A));
}


void setup(){
  Serial.begin(115200);
  L.begin();
  L.scan();
  L.reset();
  delay(100);
  //changeAddress
  delay(100);
  L.changeAddressMultiPwrEn(4, addresses, sensorPinsEN, 2);
  Serial.println(L.isOnline(0x62));
  Serial.println(L.isOnline(0x64));
  Serial.println(L.isOnline(0x66));
  Serial.println(L.isOnline(0x68));
  Serial.println(L.isOnline(0x6A));
}

void loop(){
  L.scan();
  long start = micros();
  //int total = oneShot(address);
  shotThemAll();
  long stop = micros();
  #if READINESS
  Serial.print("Business : ");
  Serial.print(stop-start);
  Serial.println(" microS ");
  //Serial.print("measure : ");
  //Serial.println(total);
  #else
  Serial.println(stop-start);
  //Serial.print(",");
  //Serial.println(total);
  #endif

}

