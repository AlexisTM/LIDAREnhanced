#include <Arduino.h>
#include <Wire.h>

class LIDAREnchanced {
  public:
    LIDAREnchanced() {}
    // Configure lasers
    void begin(bool fasti2c = false){
      Wire.begin();
      if (fasti2c) {
      #if ARDUINO >= 157
          Wire.setClock(400000UL); // Set I2C frequency to 400kHz, for the Due
      #else
          TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
      #endif
      }
    }

    void reset(byte Lidar = 0x62){
      write(Lidar, 0x00, 0x00);
      if(Lidar != 0x62)
        changeAddress(0x62, Lidar);
    }

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
    }

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
    }

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
    }

    // Check availability
    bool isBusy(byte Lidar = 0x62){
      Wire.beginTransmission((byte)Lidar);
      Wire.write(0x01);
      Wire.requestFrom((byte)Lidar, (byte)1);
      byte busyFlag = bitRead(Wire.read(), 0);
      return busyFlag != 0;
    }
    bool isOnline(byte Lidar = 0x62){
      Wire.beginTransmission(Lidar);
      if(Wire.endTransmission())
        return false;
      return true;
    }
    void whoisOnline(int number, byte * Lidars){
      for(int i = 0; i < number; i++){
        Serial.print("Lidar ");
        Serial.print(Lidars[i],HEX);
        if(isOnline(Lidars[i]))
          Serial.println(" is ONLINE");
        else
          Serial.println(" is OFFLINE");


      }
    }
    byte status(byte Lidar = 0x62){
      byte data[1] = {0};
      readbyte(Lidar, 0x01, data);
      return data[0];
    }

    // Read values
    byte async(byte Lidar = 0x62){
      write(Lidar, 0x00, 0x04);
    }

    int distance(byte Lidar = 0x62){
      byte distanceArray[2];
      readWord(Lidar, 0x8f, distanceArray);
      int distance = (distanceArray[0] << 8) + distanceArray[1];
      return (distance);
    }

    // I2C Functions
    byte write(byte Lidar, byte regAdr, byte data){
      Wire.beginTransmission(Lidar);
      Wire.write(regAdr);
      Wire.write(data);
      int nackCatcher = Wire.endTransmission();
      return nackCatcher;
    }

    byte readbyte(byte Lidar, byte regAdr, byte * data){
      Wire.beginTransmission(Lidar);
      Wire.write(regAdr);
      int nackCatcher = Wire.endTransmission();
      Wire.requestFrom(Lidar, byte(1), byte(1));
      int i = 0;
      data[0] = Wire.read();
      return nackCatcher;
    }

    byte readWord(byte Lidar, byte regAdr, byte * data){
      Wire.beginTransmission(Lidar);
      Wire.write(regAdr);
      int nackCatcher = Wire.endTransmission();
      Wire.requestFrom(Lidar, byte(2), byte(1));
      data[0] = Wire.read();
      data[1] = Wire.read();
      return nackCatcher;
    }

    // Debug informations
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
    }

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
    }
};
