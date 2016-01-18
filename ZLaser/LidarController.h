#ifndef LIDAR_CONTROLLER_H
#define LIDAR_CONTROLLER_H

#include "I2CFunctions.h"
#include "LidarObject.h"
#include "KFilter.h"
#include <Wire.h>

#define MAX_LIDARS 4

class LidarController {
  public: 
    void begin(bool fasti2c = false){
      resetToReset();
      Wire.begin();
      if (fasti2c) {
      #if ARDUINO >= 157
          Wire.setClock(400000UL); // Set I2C frequency to 400kHz, for the Due
      #else
          TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
      #endif
      }
    }

    bool add(LidarObject* _Lidar, byte _id, KFilter* _filter = NULL){
      if(_id >= MAX_LIDARS)
        return false;
      lidars[_id] = _Lidar;
      filters[_id] = _filter;
      setState(_id, NEED_CONFIGURE);
      count++;
      return true;
    }

/*******************************************************************************
  checkTimers : 
*******************************************************************************/
    void checkTimers(){
      for(byte i = 0; i < count; i++){
        if(lidars[i]->check_timer()){
          postReset(i);
          setState(i,NEED_CONFIGURE);
        }
      }
    };


/*******************************************************************************
  configure : Configure the default acquisition mode

  configuration : The configuration of the Lidar
      - 0 = basic configuration
      - 1 = faster (Do not read 3 times), bit noisier
      - 2 = low noise, low sensitivity, less false detection (Default)
      - 3 = High noise, high sensitivity
  Lidar : Address of the Lidar (0x62 by default)
*******************************************************************************/
    void configure(byte Lidar = 0, byte configuration = 2){
      switch (configuration) {
      case 0: //  Default configuration
        I2C.write(lidars[Lidar]->address, 0x00, 0x00);
        break;
      case 1: //  Set aquisition count to 1/3 default value, faster reads, slightly
        //  noisier values
        I2C.write(lidars[Lidar]->address, 0x04, 0x00);
        break;
      case 2: //  Low noise, low sensitivity: Pulls decision criteria higher
        //  above the noise, allows fewer false detections, reduces
        //  sensitivity
        I2C.write(lidars[Lidar]->address, 0x1c, 0x20);
        break;
      case 3: //  High noise, high sensitivity: Pulls decision criteria into the
        //  noise, allows more false detections, increses sensitivity
        I2C.write(lidars[Lidar]->address, 0x1c, 0x60);
        break;
      }
      setState(Lidar, CONFIGURED);
    };

/*******************************************************************************
  changeAddress : Change the address of one Lidar

returns 0 if success
        1 if error writing the serial number (byte 1)
        2 if error writing the serial number (byte 2)
        3 if error sending the Lidar address
        4 if error disabling the main address
        5 if the new Lidar address is already ON
        6 if the Lidar do not respond
*******************************************************************************/
    byte changeAddress(byte Lidar){
      byte _lidar_new = lidars[Lidar]->address;
      // Return 6 = The device do not respond
      if(!I2C.isOnline(0x62))
        return 6;
      // Return 5 = We already got an I2C device at this place
      if(I2C.isOnline(_lidar_new))
        return 5;
      /* Serial number part */
      unsigned char serialNumber[2];
      I2C.readWord(0x62, 0x96, serialNumber);
      // Return 1 = Error sending the Serial (byte 1)
      if(I2C.nackError(I2C.write(0x62, 0x18, serialNumber[0])))
        return 1;
      // Return 2 = Error sending the Serial (byte 2)
      if(I2C.nackError(I2C.write(0x62, 0x19, serialNumber[1])))
        return 2;

      // Return 3 = Error sending the Lidar address
      if(I2C.nackError(I2C.write(0x62, 0x1a, _lidar_new)))
        return 3;

      // Return 4 = Error disabling the Lidar Main address (0x62)
      if(I2C.nackError(I2C.write(0x62, 0x1e, 0x08)))
        return 4;

      return 0;
    };


/*******************************************************************************
  changeAddressMultiPwrEn : Change multiple Lidars addresses
  number : number of Lidar we would like to change the address
  newLidar : Array of new Lidars addresses
  pwrEnable : Array of new Lidars power enable lines
  configuration : The configuration wanted (2 by default)
*******************************************************************************/
    void changeAllAddresses(){
      // Shutdown them all
      for(int i = 0; i < count; i++){
        pinMode(lidars[i]->EnablePin, OUTPUT);
        delay(20);
        lidars[i]->off();
        delay(20);
      }
      // Power on them one per one,
      // Change their address & configure them
      for(int i = 0; i < count; i++){
        lidars[i]->on();
        delay(40);
        changeAddress(i);
        delay(20);
        configure(lidars[i]->configuration, lidars[i]->address);
      }
    }

/*******************************************************************************
  isBusy : check if the Lidar is in acquisition

  returns true if busy
          flse if ready to be read
*******************************************************************************/
    bool isBusy(byte Lidar = 0){
      byte _lidar = lidars[Lidar]->address;
      Wire.beginTransmission((byte)_lidar);
      Wire.write(0x01);
      Wire.requestFrom((byte)_lidar, (byte)1);
      byte busyFlag = bitRead(Wire.read(), 0);
      return busyFlag != 0;
    };

/*******************************************************************************
  status : check the status of the Lidar

  returns the status register (0x01 for version 21 of the Lidar Software)
*******************************************************************************/
    byte status(byte Lidar = 0){
      byte data[1] = {0};
      I2C.readByte(lidars[Lidar]->address, 0x01, data);
      return data[0];
    };

/*******************************************************************************
  async : start an acquisition
              - with preamp enabled
              - with DC stabilization

  returns the nack error (0 if no error)
*******************************************************************************/
    byte async(byte Lidar = 0){
      I2C.write(lidars[Lidar]->address, 0x00, 0x04);
      setState(Lidar, ACQUISITION_READY);
    };
    
/*******************************************************************************
  asyncAll
*******************************************************************************/
    void asyncAll(){
      for(byte i = 0; i < count; i++){
        async(i);
      }
    };

/*******************************************************************************
  distance : read and
              - with preamp enabled
              - with DC stabilization

  returns the distance in centimeter (-1 = error from I2C)
*******************************************************************************/
    int distance(byte Lidar, int * data){
      byte distanceArray[2];
      byte nackCatcher = I2C.readWord(lidars[Lidar]->address, 0x8f, distanceArray);
      int distance = (distanceArray[0] << 8) + distanceArray[1];
      setState(Lidar, ACQUISITION_DONE);
      *data = distance;
      return nackCatcher;
    };

    void setState(byte Lidar = 0, LIDAR_STATE _lidar_state = UNKNOWN){
      lidars[Lidar]->lidar_state = _lidar_state;
    };

    LIDAR_STATE getState(byte Lidar = 0){
      return lidars[Lidar]->lidar_state;
    };

    byte distanceAndAsync(byte Lidar, int * data){
      byte nackCatcher = distance(Lidar, data);
      // if error reading the value, try ONCE again
      if(nackCatcher)
        distance(Lidar, data);
      // Start a new acquisition
      async(Lidar);
      lidars[Lidar]->lidar_state = ACQUISITION_READY;
      return nackCatcher;
    };

    void resetToReset(){
      for(byte i = 0; i < MAX_LIDARS; i++){
        toReset[i] = 0;
      }
    }

    void addToReset(byte Lidar = 0){
      toReset[Lidar] = 1;
      preReset(Lidar);
    }

    void removeToReset(byte Lidar = 0){
      toReset[Lidar] = 0;
      postReset(Lidar);
    }

    bool checkAllToReset(){
      for(byte i = 0; i < MAX_LIDARS; i++){
        if(toReset[i]){
          switch(lidars[i]->lidar_state){
           case WAIT_AFTER_RESET : 
              return false;
           case NEED_RESET : 
              return false;
           case WAIT_AFTER_RESET_DONE : 
              break;
           default : 
              //error ? 
              break; 
          }
        }
      }
      return true;
    }

/*******************************************************************************
  reset : Reset an I2C Lidar sending an I2C packet

  Software : Write 0x00 at the 0x00 register
  Use this when you need to reset the device (the Lidar sends only wrong data)
*******************************************************************************/
    void preReset(byte Lidar = 0){
      byte _lidar = lidars[Lidar]->address;
      I2C.write(_lidar, 0x00, 0x00);
      if(_lidar != 0x62)
        changeAddress(Lidar);
      lidars[Lidar]->off();
      delayMicroseconds(10);
      lidars[Lidar]->on();
      lidars[Lidar]->timer_update();

      setState(Lidar, WAIT_AFTER_RESET);
    };

/*******************************************************************************
  postReset : 
*******************************************************************************/
    void postReset(byte Lidar = 0){
      byte _lidar = lidars[Lidar]->address;
      changeAddress(Lidar);
      configure(Lidar);
    };

    void changeAddressesToReset(){
      if(!checkAllToReset()){
        Serial.println("\n All lasers to reset are not ready");
        return;
      }

      // disable all
      for(byte i = 0; i < MAX_LIDARS; i++){
        if(toReset[i]){
          lidars[i]->disable();
        }
      }

      // enable, one per one, and change address
      for(byte i = 0; i < MAX_LIDARS; i++){
        if(toReset[i]){
          lidars[i]->enable();
          delayMicroseconds(10);
          postReset(i);
        }
      }
      resetToReset();
      return;
    }
    
    // Public vars
    // Just to make it easier
    KFilter* filters[MAX_LIDARS];

  private:
    // 0 is not to reset, 1 is to reset
    byte toReset[MAX_LIDARS];
    LidarObject* lidars[MAX_LIDARS];
    byte count = 0;
    
};


#endif
