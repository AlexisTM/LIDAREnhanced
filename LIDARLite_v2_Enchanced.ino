#include "LidarObject.h"
#include "LidarController.h"
#include "I2CFunctions.h"
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

void beginLidars(){
  // Initialisation of the lidars objects
  LX1.begin(X_LASER_EN, X_LASER_PIN, X_LASER_AD, 2, 'x');
  LX2.begin(Y_LASER_EN, Y_LASER_PIN, Y_LASER_AD, 2, 'X');
  LY1.begin(Z_LASER_EN, Z_LASER_PIN, Z_LASER_AD, 2, 'y');
  LY2.begin(W_LASER_EN, W_LASER_PIN, W_LASER_AD, 2, 'Y');

  // Initialisation of the controller
  Controller.begin(false);

  // Adding Lidars to the controller
  Controller.add(&LX1, 0, &Kx);
  Controller.add(&LX2, 1, &KX);
  Controller.add(&LY1, 2, &Ky);
  Controller.add(&LY2, 3, &KY);

  Controller.changeAllAddresses();
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
    Serial.println("\n Begin filters");
    Serial.print(data[0]);
    Serial.print("\t");
    Serial.print(data[1]);
    Serial.print("\t");
    Serial.print(data[2]);
    Serial.println("\t");
  }
}

void setup(){
  Serial.begin(115200);
  while(!Serial);
  beginLidars();
  Controller.asyncAll();
  beginFilters();
}

void loop(){
  // Check reset lasers
  // Reset lasers
  //getData
  Controller.checkAllToReset();
  //exportData();
  laserAcquisition();
  delay(20);
  // Verify data
  // 
}

void laserAcquisition(){
  for(byte i = 0; i < NUMBER_OF_LASERS; i++){
    int data = 0;
    byte nack = Controller.distanceAndAsync(i, &data);
    double dataFiltered = Controller.filters[i]->filter(data);
    Serial.print("s: ");
    Serial.print(nack, BIN);
    Serial.print("\t  ");
    Serial.print("d: ");
    Serial.print(data);
    Serial.print("\t  ");
    Serial.print("f: ");
    Serial.print(dataFiltered);
  }
  Serial.println();
}

void exportData(){
  int data = 0;
  int nack = Controller.distanceAndAsync(0, &data);
  double dataFiltered = Controller.filters[0]->filter(data);
  Serial.print(nack);
  Serial.print(",");
  Serial.print(data);
  Serial.print(",");
  Serial.println(dataFiltered);
}

void resetLasers(){
  
}
