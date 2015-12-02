#include <Arduino.h>
#include <Wire.h>

// We got a Lidar per laser. 

enum LIDAR_STATE {
  UNKNOW,            // Need to be resetted
  ACQUISITION_READY, // I started an acquisition, need someone to read it
  ACQUISITION_DONE,  // I read the data, need to start an acq again
  NEED_RESET,        // Too much outliers, need to reset
  WAIT_AFTER_RESET,  // Wait 15ms after you reset the Lidar, we are waiting in this state
  NEED_CONFIGURE     // 15ms passed, we now configure the Lidar
};

class LIDAREnchanced {
  public:
/*******************************************************************************
  Constructor
*******************************************************************************/
    LIDAREnchanced() {};

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
  reset : Reset an I2C Lidar sending an I2C packet

  Software : Write 0x00 at the 0x00 register
*******************************************************************************/
    void reset(byte Lidar = 0x62){
      write(Lidar, 0x00, 0x00);
      if(Lidar != 0x62)
        changeAddress(0x62, Lidar);
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
    void configure(byte configuration = 2, byte Lidar = 0x62){
      switch (configuration) {
      case 0: //  Default configuration
        write(Lidar, 0x00, 0x00);
        break;
      case 1: //  Set aquisition count to 1/3 default value, faster reads, slightly
        //  noisier values
        write(Lidar, 0x04, 0x00);
        break;
      case 2: //  Low noise, low sensitivity: Pulls decision criteria higher
        //  above the noise, allows fewer false detections, reduces
        //  sensitivity
        write(Lidar, 0x1c, 0x20);
        break;
      case 3: //  High noise, high sensitivity: Pulls decision criteria into the
        //  noise, allows more false detections, increses sensitivity
        write(Lidar, 0x1c, 0x60);
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
    byte changeAddress(byte Lidar, byte newLidar){
      // Return 6 = The device do not respond
      if(!isOnline(Lidar))
        return 6;
      // Return 5 = We already got an I2C device at this place
      if(isOnline(newLidar))
        return 5;
      /* Serial number part */
      unsigned char serialNumber[2];
      readWord(Lidar, 0x96, serialNumber);
      // Return 1 = Error sending the Serial (byte 1)
      if(nackError(write(Lidar, 0x18, serialNumber[0])))
        return 1;
      // Return 2 = Error sending the Serial (byte 2)
      if(nackError(write(Lidar, 0x19, serialNumber[1])))
        return 2;

      // Return 3 = Error sending the Lidar address
      if(nackError(write(Lidar, 0x1a, newLidar)))
        return 3;

      // Return 4 = Error disabling the Lidar Main address (0x62)
      if(nackError(write(Lidar, 0x1e, 0x08)))
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
    byte changeAddressMultiPwrEn(byte number, byte * newLidar, byte * pwrEnable, byte configuration = 2){
      // Shutdown them all
      for(int i = 0; i < number; i++){
        pinMode(pwrEnable[i], OUTPUT);
        delay(5);
        digitalWrite(pwrEnable[i], LOW);
        delay(5);
      }
      // Power on them one per one,
      // Change their address & configure them
      for(int i = 0; i < number; i++){
        digitalWrite(pwrEnable[i], HIGH);
        delay(20);
        changeAddress(0x62, newLidar[i]);
        delay(20);
        configure(configuration, newLidar[i]);
      }
    };

/*******************************************************************************
  isBusy : check if the Lidar is in acquisition

  returns true if busy
          flse if ready to be read
*******************************************************************************/
    bool isBusy(byte Lidar = 0x62){
      Wire.beginTransmission((byte)Lidar);
      Wire.write(0x01);
      Wire.requestFrom((byte)Lidar, (byte)1);
      byte busyFlag = bitRead(Wire.read(), 0);
      return busyFlag != 0;
    };

/*******************************************************************************
  isOnline : check if something is connected at this address

  returns true if online
          flse if offline
*******************************************************************************/
    bool isOnline(byte Lidar = 0x62){
      Wire.beginTransmission(Lidar);
      if(Wire.endTransmission())
        return false;
      return true;
    };

/*******************************************************************************
  whoisOnline : check the array of Lidar to know if they are online

  Debug only, it prints on Serial1
*******************************************************************************/
    void whoisOnline(int number, byte * Lidars){
      for(int i = 0; i < number; i++){
        Serial.print("Lidar ");
        Serial.print(Lidars[i],HEX);
        if(isOnline(Lidars[i]))
          Serial.println(" is ONLINE");
        else
          Serial.println(" is OFFLINE");
      }
    };

/*******************************************************************************
  status : check the status of the Lidar

  returns the status register (0x01 for version 21 of the Lidar Software)
*******************************************************************************/
    byte status(byte Lidar = 0x62){
      byte data[1] = {0};
      readByte(Lidar, 0x01, data);
      return data[0];
    };

/*******************************************************************************
  async : start an acquisition
              - with preamp enabled
              - with DC stabilization

  returns the nack error (0 if no error)
*******************************************************************************/
    byte async(byte Lidar = 0x62){
      write(Lidar, 0x00, 0x04);
    };

/*******************************************************************************
  distance : read and
              - with preamp enabled
              - with DC stabilization

  returns the distance in centimeter (-1 = error from I2C)
*******************************************************************************/
    int distance(byte Lidar = 0x62){
      byte distanceArray[2];
      readWord(Lidar, 0x8f, distanceArray);
      int distance = (distanceArray[0] << 8) + distanceArray[1];
      return (distance);
    };

/*******************************************************************************
  write : write a byte to one I2C device at a certain address

  Lidar : the I2C device address
  regAdr : The I2C foreign register
  data : The data to put in the regAdr register of the i2c (Lidar) device

  returns the nack packet
*******************************************************************************/
    byte write(byte Lidar, byte regAdr, byte data){
      Wire.beginTransmission(Lidar);
      Wire.write(regAdr);
      Wire.write(data);
      byte nackCatcher = Wire.endTransmission();
      return nackCatcher;
    };

/*******************************************************************************
  readByte : read one 8-bit byte from one I2C device

  Lidar : the I2C device address
  regAdr : The I2C foreign register
  data : The data array where to put data

  returns the nack packet
*******************************************************************************/
    byte readByte(byte Lidar, byte regAdr, byte * data){
      Wire.beginTransmission(Lidar);
      Wire.write(regAdr);
      byte nackCatcher = Wire.endTransmission();
      Wire.requestFrom(Lidar, byte(1), byte(1));
      data[0] = Wire.read();
      return nackCatcher;
    };

/*******************************************************************************
  readWord : read two 8-bit byte from one I2C device

  Lidar : the I2C device address
  regAdr : The I2C foreign register
  data : The data array where to put data

  returns the nack packet
*******************************************************************************/
    byte readWord(byte Lidar, byte regAdr, byte * data){
      Wire.beginTransmission(Lidar);
      Wire.write(regAdr);
      int nackCatcher = Wire.endTransmission();
      Wire.requestFrom(Lidar, byte(2), byte(1));
      data[0] = Wire.read();
      data[1] = Wire.read();
      return nackCatcher;
    };

/*******************************************************************************
  scan : debug function used to show which device is currently on the I2C bus
*******************************************************************************/
    void scan(){
      delay(1000);
      byte error, address;
      int nDevices;
      Serial.println("Scanning...");

      nDevices = 0;
      for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
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

  error : the byte of error from the I2C device

  returns the error
*******************************************************************************/
    byte nackError(byte error) {
      if (error) {
        Serial.println("NackError");
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
        }
      }
      return error;
    };
};
