/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <laserpack/distance.h>
#include <laserpack/init.h>
#include <laserpack/req_reset.h>
#include <laserpack/req_init.h>

ros::NodeHandle nh;

laserpack::distance   distance_msg;
laserpack::init       init_msg;
laserpack::req_reset  req_reset_msg;
laserpack::req_init   req_init_msg;

void req_reset_func(const laserpack::req_reset& msg){

}

void req_reset_func(const laserpack::req_init& msg){
  
}


ros::Publisher distpub("distance", &distance_msg);
ros::Publisher initpub("init", &init_msg);
ros::Subscriber<laserpack::req_reset> subReset("req_reset", &req_reset_func );
ros::Subscriber<laserpack::req_init> subInit("req_init", &req_reset_func );

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(distpub);
  nh.advertise(initpub);
  nh.subscribe(subReset);
  nh.subscribe(subInit);
}

void loop()
{
  nh.spinOnce();
  delay(1);
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}