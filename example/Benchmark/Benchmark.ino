#include <Arduino.h>

/*------------------------------------------------------------------------------

  Benchmark script
  http://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf

------------------------------------------------------------------------------*/

#include <Wire.h>
#include <I2CFunctions.h> 
#include <LidarObject.h>
#include <LidarController.h>

#define LIDARLITE_ADDR_DEFAULT 0x62

#define WIRE400K true
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

#define NUMBER_OF_LASERS 5


static LidarController Controller;
static LidarObject LZ1;
static LidarObject LZ2;
static LidarObject LZ3;
static LidarObject LZ4;

long now, last;
int errors = 0;
long measureCount = 0;

#define N_MEASURE_PROFILE 1000.0

void setup()
{
  Serial.begin(115200);
  LZ1.begin(Z1_LASER_EN, Z1_LASER_PIN, Z1_LASER_TRIG, Z1_LASER_AD, 2, DISTANCE, 'A');
  LZ2.begin(Z2_LASER_EN, Z2_LASER_PIN, Z2_LASER_TRIG, Z2_LASER_AD, 2, DISTANCE, 'B');
  LZ3.begin(Z3_LASER_EN, Z3_LASER_PIN, Z3_LASER_TRIG, Z3_LASER_AD, 2, DISTANCE, 'C');
  LZ4.begin(Z4_LASER_EN, Z4_LASER_PIN, Z4_LASER_TRIG, Z4_LASER_AD, 2, DISTANCE, 'D');

  LZ1.setCallbackDistance(&distance_callback);
  LZ2.setCallbackDistance(&distance_callback);
  LZ3.setCallbackDistance(&distance_callback);
  LZ4.setCallbackDistance(&distance_callback);
  
  Controller.add(&LZ1, 0);
  Controller.add(&LZ2, 1);
  Controller.add(&LZ3, 2);
  Controller.add(&LZ4, 3);
  delay(100);
  Controller.begin(WIRE400K);
  delay(100);
}

void distance_callback(LidarObject* self){
  measureCount++;
  isError(self);
  /*Serial.print(self->name);
  Serial.print(":");
  Serial.println(self->distance);*/
}

void loop()
{
  Serial.println("Starting...");
  
  for(int conf = 0; conf < 6; conf++){
    //reconfigure(conf);
    last = micros();
    for(long i = 0; i < 1000; i++){
      Controller.spinOnce(false);
    }
    now = micros();
    Serial.print("Configuration ");
    Serial.print(conf);
    Serial.print(" spinOnce() : ");
    Serial.print(measureCount*1000000.0/double(now-last));
    Serial.print("Hz, errors : ");
    Serial.print(errors);
    Serial.print(" on ");
    Serial.print(measureCount);
    Serial.println(" measurements ");
    errors = 0;
    measureCount=0;
  }
}

void isError(LidarObject* self){
  if(self->distance > 500) {
    errors++;
    Serial.print(self->name);
    Serial.print(":");
    Serial.println(self->distance);
  } else if(self->distance < 50){
    errors++;
    Serial.print(self->name);
    Serial.print(":");
    Serial.println(self->distance);
  }
}

void reconfigure(int conf){
  for(int j = 0; j < 4; j++){
    Controller.lidars[j]->configuration = conf;
    Controller.resetLidar(j);
  }
  
  for(int j = 0; j < 4; j++){
    Controller.spinOnce();
    delay(20);
    Controller.spinOnce();
    delay(20);
    Controller.spinOnce();
  }
}
