#ifndef I2C_FUNCTIONS_H
#define I2C_FUNCTIONS_H

#include <Arduino.h>
// #include <Wire.h>
#include <i2c_t3.h>

#define STOP_CONDITION_I2C true
class I2CFunctions {
  public:
  	I2CFunctions() {};
/*******************************************************************************
  begin : Begin the I2C master device

  If fasti2c is true, use 400kHz I2C
*******************************************************************************/
    void begin(bool fasti2c = false){
      Wire.begin();
      if (fasti2c) {
      #if ARDUINO >= 157
          Wire.setClock(400000UL); // Set I2C frequency to 400kHz, for the Due
      #else
          TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
      #endif
      }
    };


/*******************************************************************************
  isOnline : check if something is connected at this address

  returns true if online
          flse if offline
*******************************************************************************/
    bool isOnline(uint8_t Device = 0x62){
      Wire.beginTransmission(Device);
      if(Wire.endTransmission(STOP_CONDITION_I2C))
        return false;
      return true;
    };

/*******************************************************************************
  whoisOnline : check the array of Device to know if they are online

  Debug only, it prints on Serial1
*******************************************************************************/
    void whoisOnline(int number, uint8_t * Devices){
      for(int i = 0; i < number; i++){
        Serial.print("Device ");
        Serial.print(Devices[i],HEX);
        if(isOnline(Devices[i]))
          Serial.println(" is ONLINE");
        else
          Serial.println(" is OFFLINE");
      }
    };

/*******************************************************************************
  write : write a uint8_t to one I2C device at a certain address

  Device : the I2C device address
  regAdr : The I2C foreign register
  data : The data to put in the regAdr register of the i2c (Device) device

  returns the nack packet
*******************************************************************************/
    uint8_t write(uint8_t Device, uint8_t regAdr, uint8_t data){
      Wire.beginTransmission(Device);
      Wire.write(regAdr);
      Wire.write(data);
      uint8_t nackCatcher = Wire.endTransmission(STOP_CONDITION_I2C);
      return nackCatcher;
    };

/*******************************************************************************
  readByte : read one 8-bit uint8_t from one I2C device

  Device : the I2C device address
  regAdr : The I2C foreign register
  data : The data array where to put data

  returns the nack packet
*******************************************************************************/
    uint8_t readByte(uint8_t Device, uint8_t regAdr, uint8_t * data){
      Wire.beginTransmission(Device);
      Wire.write(regAdr);
      uint8_t nackCatcher = Wire.endTransmission(STOP_CONDITION_I2C);
      Wire.requestFrom(Device, uint8_t(1), uint8_t(1));
      data[0] = Wire.read();
      return nackCatcher;
    };

/*******************************************************************************
  readWord : read two 8-bit uint8_t from one I2C device

  Device : the I2C device address
  regAdr : The I2C foreign register
  data : The data array where to put data

  returns the nack packet
*******************************************************************************/
    uint8_t readWord(uint8_t Device, uint8_t regAdr, uint8_t * data){
      Wire.beginTransmission(Device);
      Wire.write(regAdr);
      int nackCatcher = Wire.endTransmission(STOP_CONDITION_I2C);
      Wire.requestFrom(Device, uint8_t(2), uint8_t(1));
      data[0] = Wire.read();
      data[1] = Wire.read();
      return nackCatcher;
    };

/*******************************************************************************
  scan : debug function used to show which device is currently on the I2C bus
*******************************************************************************/
    void scan(){
      uint8_t error, address;
      int nDevices;
      Serial.println("Scanning...");

      nDevices = 0;
      for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission(STOP_CONDITION_I2C);
        if (error == 0) {
          Serial.print("I2C device found at address 0x");
          if (address<16)
            Serial.print("0");
          Serial.print(address,HEX);
          Serial.println("  !");
          nDevices++;
        }
        else if (error==4) {
          Serial.print("Unknow error at address 0x");
          if (address<16)
            Serial.print("0");
          Serial.println(address,HEX);
        }
      }
      if (nDevices == 0)
        Serial.println("No I2C devices found\n");
      else
        Serial.println("done\n");
    };

/*******************************************************************************
  nackError : debug function, show the error if we got a NACK error

  error : the uint8_t of error from the I2C device

  returns the error
*******************************************************************************/
    uint8_t nackError(uint8_t error) {
      if (error) {
        Serial.print("NackError : ");
        switch (error) {
          case 1:
            Serial.println("Message is too long in transmit buffer");
            break;
          case 2:
            Serial.println("NACK on transmit address");
            break;
          case 3:
            Serial.println("NACK on transmit data");
            break;
          case 4:
            Serial.println("Other error");
            break;
          default:
            Serial.println("No Error");
        }
      }
      return error;
    };
};


I2CFunctions I2C;
extern I2CFunctions I2C;

#endif
