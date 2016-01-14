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
#define X_LASER_PIN 3
#define Y_LASER_PIN 5
#define Z_LASER_PIN 7
#define W_LASER_PIN 9
// Defines power enable lines of laser
#define X_LASER_EN 10
#define Y_LASER_EN 11
#define Z_LASER_EN 12
#define W_LASER_EN 13
//Define address of lasers
//Thoses are written during initialisation
// default address : 0x62
#define X_LASER_AD 0x64
#define Y_LASER_AD 0x66
#define Z_LASER_AD 0x68
#define W_LASER_AD 0x6A

#define NUMBER_OF_LASERS 4

#define READINESS true

// Lidars
static LidarController Controller;
static LidarObject LX1;
static LidarObject LX2;
static LidarObject LY1;
static LidarObject LY2;
static KFilter Kx;
static KFilter KX;
static KFilter Ky;
static KFilter KY;

// ROS communication
ros::NodeHandle nh;
laserpack::distance   distance_msg;
ros::Publisher distpub("/acquisition/distance", &distance_msg);
//laserpack::init   init_msg;
//ros::Publisher initpub("/acquisition/init", &init_msg);


void beginLidars(){
  // Initialisation of the lidars objects
  LX1.begin(X_LASER_EN, X_LASER_PIN, X_LASER_AD, 2, 'x');
  LX2.begin(Y_LASER_EN, Y_LASER_PIN, Y_LASER_AD, 2, 'X');
  LY1.begin(Z_LASER_EN, Z_LASER_PIN, Z_LASER_AD, 2, 'y');
  LY2.begin(W_LASER_EN, W_LASER_PIN, W_LASER_AD, 2, 'Y');
  // Initialisation of the controller
  Controller.begin(WIRE400K);

  // Adding Lidars to the controller
  Controller.add(&LX1, 0, &Kx);
  Controller.add(&LX2, 1, &KX);
  Controller.add(&LY1, 2, &Ky);
  Controller.add(&LY2, 3, &KY);

  Controller.changeAllAddresses();
  delay(15);
 // Controller.changeAllAddresses();
 // delay(10);
}


void beginROSComm(){
  nh.initNode();
  nh.advertise(distpub);
//  nh.advertise(initpub);
  pinMode(13, OUTPUT);
}

void beginFilters(){  

  // Initialisation of the lidars objects
  for(byte i = 0; i < 4; i++){
    int data[3] = {0,0,0};
    int temp = 0;
    delay(20);
    Controller.distanceAndAsync(i, &data[0]);
    delay(20);
    Controller.distanceAndAsync(i, &data[1]);
    delay(20);
    Controller.distanceAndAsync(i, &data[2]);

    // Sort the 3 data
    if(data[1] < data[0]){
      temp = data[0];
      data[0] = data[1];
      data[1] = temp;
    }
    if(data[2] < data[1]){
      temp = data[1];
      data[1] = data[2];
      data[2] = temp;
    }
    if(data[1] < data[0]){
      temp = data[0];
      data[0] = data[1];
      data[1] = temp;
    }
    if(data[2] < data[1]){
      temp = data[1];
      data[1] = data[2];
      data[2] = temp;
    }

    Controller.filters[i]->begin(data[1], 0, 200, 1, 20);
  }
}

void setup(){
  Serial.begin(57600);
  while(!Serial);
  beginROSComm();
  beginLidars();
  Controller.asyncAll();
  beginFilters();
}

void loop(){
  nh.spinOnce();
  // Check reset lasers
  // Reset lasers
  //getData
  Controller.checkAllToReset();
  //exportData();
  //laserAcquisition();
  //laserPublish();
  outputASCII();
  delay(5);
  // Verify data
}

void laserPublish(){
  static int16_t dataOut[6] = {0,0,0,0,0,0};
  static uint8_t statusOut[6] = {0,0,0,0,0,0};
  distance_msg.lasers_length = 6;
  distance_msg.status_length = 6;
  distance_msg.lasers = dataOut;
  distance_msg.status = statusOut;
  
  for(byte i = 0; i < 4; i++){
    int data = 0;
    bool isItAnOutlier = false;
    statusOut[i] = Controller.distanceAndAsync(i, &data);
    dataOut[i] =  Controller.filters[i]->filter(data, &isItAnOutlier);
    if(isItAnOutlier){
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
  for(byte i = 0; i < 4; i++){
    if(i != 0)
      Serial.print(",");
    int data = 0;
    Controller.distanceAndAsync(i, &data);
    Serial.print(data);
  }
  Serial.print("\n");
}

void resetLasers(){
  
}


