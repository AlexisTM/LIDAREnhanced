#include <Arduino.h>
#include <Wire.h>
#include "I2CFunctions.h"
// We got a Lidar per laser. 

#ifndef LIDAR_OBJECT_H
#define LIDAR_OBJECT_H

enum LIDAR_STATE {
  UNKNOWN,            // Need to be resetted
  NEED_CONFIGURE,     // 15ms passed, we now configure the Lidar
  CONFIGURED,          // We configured the laser
  ACQUISITION_READY, // I started an acquisition, need someone to read it
  ACQUISITION_DONE,  // I read the data, need to start an acq again
  NEED_RESET,        // Too much outliers, need to reset
  WAIT_AFTER_RESET,  // Wait 15ms after you reset the Lidar, we are waiting in this state
  WAIT_AFTER_RESET_DONE
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
    void begin(byte _EnablePin = 2, byte _ModePin = 1, byte _Lidar = 0x62, byte _configuration = 2,char _name = 'A'){
      configuration = _configuration;
      address = _Lidar;
      lidar_state = UNKNOWN;
      EnablePin = _EnablePin;
      ModePin = _ModePin;
      name = _name;
    };


/*******************************************************************************
  setName : set the One char long name

  name is a one char long name 
*******************************************************************************/
    void setName(char _name){
      name = _name;
    };

/*******************************************************************************
  getName : get the One char long name

  return name which is a one char long name 
*******************************************************************************/
    char getName(char _name){
      return name;
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
  on : Power On the device
*******************************************************************************/
    void enable(){
      digitalWrite(ModePin, HIGH);
    };

/*******************************************************************************
  off : Power Off the device
*******************************************************************************/
    void disable(){
      digitalWrite(ModePin, LOW);
    };
    
    void timer_update(){
      timeReset = micros();
    }

    bool check_timer(){
      if(lidar_state == WAIT_AFTER_RESET_DONE)
        return true;
      if(lidar_state == WAIT_AFTER_RESET){
        if(micros() - timeReset > 16000) {
          lidar_state = WAIT_AFTER_RESET_DONE;
          return true;
        }
      }
      return false;
    }

    long timeReset = 0;
    byte configuration = 2;
    byte address = 0x62;
    LIDAR_STATE lidar_state = UNKNOWN;
    byte EnablePin = 2;
    byte ModePin = 1;
    char name;
};

#endif
