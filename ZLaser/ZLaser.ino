#include "LidarObject.h"
#include "LidarController.h"
#include "I2CFunctions.h"

// Only if multiple UART port board
//#define USE_USBCON
#include <ros.h>
#include <laserpack/distance.h>
//#include <laserpack/init.h>

#include <Wire.h>
#define WIRE400K true
/*** Defines : CONFIGURATION ***/
// Defines laser ready data
#define Z1_LASER_PIN 3
#define Z2_LASER_PIN 5
// Defines power enable lines of laser
#define Z1_LASER_EN 11
#define Z2_LASER_EN 2
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

// ROS communication
ros::NodeHandle nh;
laserpack::distance   distance_msg;
ros::Publisher distpub("/lasers/raw", &distance_msg);

void beginLidars(){
  // Initialisation of the lidars objects
  LZ1.begin(Z1_LASER_EN, Z1_LASER_PIN, Z1_LASER_AD, 2, 'z');
  LZ2.begin(Z2_LASER_EN, Z2_LASER_PIN, Z2_LASER_AD, 2, 'Z');
  // Initialisation of the controller
  Controller.begin(WIRE400K);

  // Adding Lidars to the controller
  Controller.add(&LZ1, 0);
  Controller.add(&LZ2, 1);

  Controller.changeAllAddresses();
  delay(15);
 // Controller.changeAllAddresses();
 // delay(10);
}


void beginROSComm(){
  nh.initNode();
  nh.advertise(distpub);
  pinMode(13, OUTPUT);
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
  beginROSComm();
  beginLidars();
  Controller.asyncAll();
}

void loop(){
  nh.spinOnce();
  //outputASCII();
  laserPublish();
  delay(4); // 4 ms = 250Hz
}

void laserPublish(){
  static int16_t dataOut[2] = {0,0};
  static uint8_t statusOut[2] = {0,0};
  distance_msg.lasers_length = 2;
  distance_msg.status_length = 2;
  distance_msg.lasers = dataOut;
  distance_msg.status = statusOut;
  
  for(byte i = 0; i < NUMBER_OF_LASERS; i++){
    int data = 0;
    bool isItAnOutlier = false;
    statusOut[i] = Controller.distanceAndAsync(i, &data);
    dataOut[i] = data;
    if(!isPerfect(data)){
      statusOut[i] = statusOut[i] | 0x80;
    }
  }
 
  distpub.publish( &distance_msg );
}

void laserAcquisition(){
  for(byte i = 0; i < NUMBER_OF_LASERS; i++){
    int data = 0;
    byte nack = Controller.distanceAndAsync(i, &data);
    bool isItAnOutlier = false;
    double dataFiltered = Controller.filters[i]->filter(data, &isItAnOutlier);
    Serial.print("s: ");
    Serial.print(nack, BIN);
    Serial.print("\t  ");
    Serial.print("d: ");
    Serial.print(data);
    Serial.print("\t  ");
  }
  Serial.println();
}

void exportData(){
  int data = 0;
  bool isItAnOutlier = false;
  int nack = Controller.distanceAndAsync(0, &data);
  
  double dataFiltered = Controller.filters[0]->filter(data, &isItAnOutlier);
  Serial.print(isItAnOutlier);
  Serial.print(",");
  Serial.print(nack);
  Serial.print(",");
  Serial.print(data);
  Serial.print(",");
  Serial.println(dataFiltered);
}

void outputASCII(){
  // output as : 
  // int,int,int,int,int,int\n
  int data = 0;
  for(byte i = 0; i < NUMBER_OF_LASERS; i++){
    if(i != 0)
      Serial.print(",");
    int data = 0;
    Controller.distanceAndAsync(i, &data);
    Serial.print(data);
  }
  Serial.print("\n");
}

