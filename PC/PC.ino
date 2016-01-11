//#include "PacketComm.h"
//#define USE_USBCON
#include <ros.h>
#include <sensor_msgs/LaserEcho.h>
#include <std_msgs/Empty.h>

float data[6] = {10,20,30,50,60,50};

ros::NodeHandle nh;

void InitCb(const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  for(byte i = 0; i<6; i++){
    data[i] = data[i]+1.1;
  }
}

sensor_msgs::LaserEcho   laserEcho_msg;

ros::Subscriber<std_msgs::Empty> initsub("/acquisition/init", InitCb );
ros::Publisher distpub("/acquisition/echoes", &laserEcho_msg);

void setup() {
  nh.initNode();
  nh.advertise(distpub);
  nh.subscribe(initsub);
  pinMode(13, OUTPUT);
}

unsigned long now = millis();
unsigned long last = millis();
unsigned long interval = 1000;


boolean status = false;

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
  laserEcho_msg.echoes_length = 6;
  laserEcho_msg.echoes = data;
  laserEcho_msg.st_echoes = 1.1;
  distpub.publish( &laserEcho_msg );
}
