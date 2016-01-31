#ifndef LIDAR_CONTROLLER_H
#define LIDAR_CONTROLLER_H

#include "I2CFunctions.h"
#include "LidarObject.h"
#include <Wire.h>

// LASER MODES



// READ Registers
#define STATUS_REGISTER           0x01
#define SIGNAL_STRENGH_REGISTER   0x0e
#define ERROR_REGISTER            0x40
#define MEASURED_VALUE_REGISTER   0x8f
#define READ_SERIAL_REGISTERS     0x96

// WRITE Registers
#define CONTROL_REGISTER          0x00
#define SERIAL1_REGISTER          0x18
#define SERIAL2_REGISTER          0x19
#define ADDRESS_REGISTER          0x1a
#define PARTY_LINE_REGISTER       0x1e
#define COMMAND_REGISTER          0x40
#define SCALE_VELOCITY_REGISTER   0x45

// Values
#define INITIATE_VALUE            0x04
#define PARTY_LINE_ON             0x00
#define PARTY_LINE_OFF            0x08

// Busyflag from STATUS_REGISTER
#define BUSYFLAG_READY_VALUE      0x00

// Wait between I2C transactions in µs
// One bit every 10µs (2.5µs in 400kHz)
// Wait at least 5 bits to wait for slave answer
#define I2C_WAIT                  50

#define MAX_LIDARS                8

class LidarController {
  public:
    /*******************************************************************************
      begin :
    *******************************************************************************/
    void begin(bool fasti2c = false) {
      Wire.begin();
      if (fasti2c) {
#if ARDUINO >= 157
        Wire.setClock(400000UL); // Set I2C frequency to 400kHz, for the Due
#else
        TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif
      }
    }

    /*******************************************************************************
      add :
    *******************************************************************************/
    bool add(LidarObject* _Lidar, byte _id) {
      if (_id >= MAX_LIDARS)
        return false;
      lidars[_id] = _Lidar;
      setState(_id, NEED_RESET);
      count++;
      return true;
    }

    /*******************************************************************************
      checkTimers :
    *******************************************************************************/
    void checkTimers() {
      for (byte i = 0; i < count; i++) {
        if (lidars[i]->check_timer()) {
          postReset(i);
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
    void configure(byte Lidar = 0, byte configuration = 2) {
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
    byte changeAddress(byte Lidar) {
      byte _lidar_new = lidars[Lidar]->address;
      // Return 6 = The device do not respond
      if (!I2C.isOnline(0x62))
        return 6;
      // Return 5 = We already got an I2C device at this place
      if (I2C.isOnline(_lidar_new))
        return 5;
      /* Serial number part */
      unsigned char serialNumber[2];
      I2C.readWord(0x62, 0x96, serialNumber);
      // Return 1 = Error sending the Serial (byte 1)
      if (I2C.nackError(I2C.write(0x62, 0x18, serialNumber[0])))
        return 1;
      // Return 2 = Error sending the Serial (byte 2)
      if (I2C.nackError(I2C.write(0x62, 0x19, serialNumber[1])))
        return 2;

      // Return 3 = Error sending the Lidar address
      if (I2C.nackError(I2C.write(0x62, 0x1a, _lidar_new)))
        return 3;

      // Return 4 = Error disabling the Lidar Main address (0x62)
      if (I2C.nackError(I2C.write(0x62, 0x1e, 0x08)))
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
    void changeAllAddresses() {
      // Shutdown them all
      for (int i = 0; i < count; i++) {
        pinMode(lidars[i]->EnablePin, OUTPUT);
        delay(20);
        lidars[i]->off();
        delay(20);
      }
      // Power on them one per one,
      // Change their address & configure them
      for (int i = 0; i < count; i++) {
        lidars[i]->on();
        delay(40);
        changeAddress(i);
        delay(20);
        configure(i);
      }
    }

    /*******************************************************************************
      isBusy : check if the Lidar is in acquisition

      returns true if busy
              flse if ready to be read
    *******************************************************************************/
    bool isBusy(byte Lidar = 0) {
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
    byte status(byte Lidar = 0) {
      byte data[1] = {150};
      I2C.readByte(lidars[Lidar]->address, 0x01, data);
      return data[0];
    };

    /*******************************************************************************
      async : start an acquisition
                  - with preamp enabled
                  - with DC stabilization

      returns the nack error (0 if no error)
    *******************************************************************************/
    byte async(byte Lidar = 0) {
      I2C.write(lidars[Lidar]->address, 0x00, 0x04);
    };

    /*******************************************************************************
      asyncAll
    *******************************************************************************/
    void asyncAll() {
      for (byte i = 0; i < count; i++) {
        async(i);
      }
    };

    /*******************************************************************************
      distance :
        - Wait the status Ready flag
        - Read the measured value
        - Counter to return -1 if failed to return the right value

      returns the distance in centimeter (-1 = error from I2C)
    *******************************************************************************/
    int distance(byte Lidar, int * data) {
      byte distanceArray[2];
      byte nackCatcher = I2C.readWord(lidars[Lidar]->address, 0x8f, distanceArray);
      int distance = (distanceArray[0] << 8) + distanceArray[1];
      *data = distance;
      return nackCatcher;
    };

    /*******************************************************************************
      setState :
    *******************************************************************************/
    void setState(byte Lidar = 0, LIDAR_STATE _lidar_state = NEED_CONFIGURE) {
      lidars[Lidar]->lidar_state = _lidar_state;
    };

    /*******************************************************************************
      getState :
    *******************************************************************************/
    LIDAR_STATE getState(byte Lidar = 0) {
      return lidars[Lidar]->lidar_state;
    };

    /*******************************************************************************
      distanceAndAsync :
    *******************************************************************************/
    byte distanceAndAsync(byte Lidar, int * data) {
      byte nackCatcher = distance(Lidar, data);
      // if error reading the value, try ONCE again
      if (nackCatcher)
        distance(Lidar, data);
      // Start a new acquisition
      async(Lidar);
      return nackCatcher;
    };

    /*******************************************************************************
      resetLidar :
    *******************************************************************************/
    void resetLidar(byte Lidar = 0) {
      lidars[Lidar]->off();
      setState(Lidar, NEED_RESET);
    }

    /*******************************************************************************
      preReset :
    *******************************************************************************/
    void preReset(byte Lidar = 0) {
      resetOngoing = true;
      lidars[Lidar]->on();
      lidars[Lidar]->timer_update();
    };

    /*******************************************************************************
      postReset :
    *******************************************************************************/
    void postReset(byte Lidar = 0) {
      byte _lidar = lidars[Lidar]->address;
      changeAddress(Lidar);
      resetOngoing = false;
    };

    /*******************************************************************************
      spinOnce :
    *******************************************************************************/
    void spinOnce() {
      // Handle everything
      for (int8_t i = count - 1; i >= 0; i--) {
        Serial.print("Laser ");
        Serial.print(i);  
        switch (getState(i)) {
          case NEED_CONFIGURE:
            Serial.println(" NEED_CONFIGURE");
            configure(i);
            setState(i, ACQUISITION_READY);
            break;
          case ACQUISITION_READY:
            Serial.println(" ACQUISITION_READY");
            async(i);
            Serial.print("isOnline :");
            Serial.println(I2C.isOnline(i));
            
            setState(i, ACQUISITION_PENDING);
            break;
          case ACQUISITION_PENDING:
            Serial.println(" ACQUISITION_PENDING");
            // Get the status bit, if 0 => Acquisition is done            
            if (bitRead( status(i), 0) == 0) {
              int data = 0;
              distanceAndAsync(i, &data);
              Serial.println(data);
          
              distances[i] = data;
              setState(i, ACQUISITION_READY);
            }

            break;
          case ACQUISITION_DONE:
            Serial.println(" ACQUISITION_DONE");

            break;
          case NEED_RESET:
            Serial.println(" NEED_RESET");
            if (!resetOngoing) {
              preReset(i);
              setState(i, RESET_PENDING);
            }
            break;
          case RESET_PENDING:
            Serial.println(" RESET_PENDING");
            // Check the timer, if done, laser is ready to reset, change state
            if (lidars[i]->check_timer()) {
              postReset(i);
              setState(i, NEED_CONFIGURE);
            }
            break;
          default:
            break;
        }
      }
    };
    /*******************************************************************************
      nextToReset :
    *******************************************************************************/
    int nextToReset() {
      for (int8_t i = count - 1; i >= 0; i--) {
        if (getState(i) == NEED_RESET)
          return i;
      }
      return -1;
    }

    int distances[MAX_LIDARS];
  private:
    bool resetOngoing = false;
    LidarObject* lidars[MAX_LIDARS];
    byte count = 0;
};


#endif
