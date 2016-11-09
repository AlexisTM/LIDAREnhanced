#include <Arduino.h>
#include <Wire.h>
#include "I2CFunctions.h"
// We got a Lidar object per laser. 

#ifndef LIDAR_OBJECT_H
#define LIDAR_OBJECT_H

enum LIDAR_STATE {
  SHUTING_DOWN = 240,       // Shutdown the laser to reset it
  NEED_RESET = 48,          // Too much outliers, need to reset
  RESET_PENDING = 80,       // Wait 15ms after you reset the Lidar, we are waiting in this state
  NEED_CONFIGURE = 144,     // 15ms passed, we now configure the Lidar
  ACQUISITION_READY = 32,   // I started an acquisition, need someone to read it
  ACQUISITION_PENDING = 64, // The acquisition in on progress
  ACQUISITION_DONE = 128    // I read the data, need to start an acq again
};

enum LIDAR_MODE {
  NONE = 0,
  DISTANCE = 1,
  VELOCITY = 2,
  DISTANCE_AND_VELOCITY = 3
};

class LidarObject {
  public:
/*******************************************************************************
  Constructor
*******************************************************************************/
    LidarObject() {};
/*******************************************************************************
  begin : Begin the I2C master device

  If fasti2c is true, use 400kHz I2C
*******************************************************************************/
    void begin(uint8_t _EnablePin = 2, uint8_t _ModePin = 1, uint8_t _Lidar = 0x62, uint8_t _configuration = 2,  LIDAR_MODE _mode = DISTANCE, char _name = 'A'){
      pinMode(_EnablePin, OUTPUT);
      mode = _mode;
      configuration = _configuration;
      address = _Lidar;
      EnablePin = _EnablePin;
      ModePin = _ModePin;
      name = _name;
    };

/*******************************************************************************
  on : Power On the device
*******************************************************************************/
    void on(){
      digitalWrite(EnablePin, HIGH);
    };

/*******************************************************************************
  off : Power Off the device
*******************************************************************************/
    void off(){
      digitalWrite(EnablePin, LOW);
    };

/*******************************************************************************
  enable : ask for PWM reading and allow continuous readings
*******************************************************************************/
    void enable(){
      digitalWrite(ModePin, HIGH);
    };

/*******************************************************************************
  disable : stops PWM reading and allow continuous readings
*******************************************************************************/
    void disable(){
      digitalWrite(ModePin, LOW);
    };

/*******************************************************************************
  timerUpdate : Update the timer to the current time to start the timer.
*******************************************************************************/
    void timerUpdate(){
      timeReset = micros();
    };

/*******************************************************************************
  checkTimer : Check the reset timer to see if the laser is correctly resetted

  The laser takes 20ms to reset
*******************************************************************************/
    bool checkTimer(){
      if(lidar_state != RESET_PENDING)
        return true;

      return (micros() - timeReset > 20000);
    };

/*******************************************************************************
  resetNacksCount : The nack counter makes the Arduino able to know if a laser 
  needs to be resetted
*******************************************************************************/
    bool resetNacksCount(){
      nacksCount = 0;
    };

/*******************************************************************************
  setCallback : The nack counter makes the Arduino able to know if a laser 
  needs to be resetted
*******************************************************************************/
    void setCallback(void (*_callback)(int32_t, int32_t, uint8_t, uint8_t)){
      notify_new_data = _callback;
    };

    int last_distance = -1;
    int distance = -1;
    int velocity = 0;
    uint8_t strength = 0;

    uint8_t nacksCount = 0;
    unsigned long timeReset = 0;
    uint8_t configuration;
    uint8_t address;
    uint8_t EnablePin;
    uint8_t ModePin;
    char name;
    LIDAR_STATE lidar_state = NEED_RESET;
    LIDAR_MODE mode = DISTANCE;
    void (*notify_new_data)(int32_t, int32_t, uint8_t, uint8_t);
};

#endif
