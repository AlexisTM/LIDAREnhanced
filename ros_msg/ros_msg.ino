/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */
#define USE_USBCON
#include <ros.h>
#include <laserpack/distance.h>

unsigned long now = millis();
unsigned long last = millis();
unsigned long interval = 1000;
boolean status = false;

ros::NodeHandle nh;

laserpack::distance   distance_msg;

ros::Publisher distpub("/acquisition/distance", &distance_msg);

void setup()
{
  nh.initNode();
  nh.advertise(distpub);
  pinMode(13, OUTPUT);
}

void loop()
{
  now = millis();
  if(now - last >=  interval){
    last = now;
    digitalWrite(13, status);
    status = status == false;
    publish();
  }
  nh.spinOnce();
}

void publish(){
  int16_t dataX[2] = {12,32}; 
  int16_t dataY[2] = {23,34};
  int16_t dataZ[2] = {65,12};
  distance_msg.X = dataX;
  distance_msg.X_length = 2;
  distance_msg.Y = dataY;
  distance_msg.Y_length = 2;
  distance_msg.Z = dataZ;
  distance_msg.Z_length = 2;
  distpub.publish( &distance_msg );
}

