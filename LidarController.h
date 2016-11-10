#ifndef LIDAR_CONTROLLER_H
#define LIDAR_CONTROLLER_H

#include "I2CFunctions.h"
#include "LidarObject.h"
#include <Wire.h>

// LASER MODES


// Registers are separeted between READ & WRITE registers.
// Indeed the result reading or writing to the same internal register does not affect
// the Lidar the same way
// All registers are not used here, but they are ready to be used in newer versions

// READ Registers
// Those are registers we only READ from
#define STATUS_REGISTER           0x01
#define SIGNAL_STRENGTH_REGISTER  0x0e
#define ERROR_REGISTER            0x40
#define MEASURED_VALUE_REGISTER   0x8f
#define READ_SERIAL_REGISTERS     0x96

// WRITE Registers
// Those are register we only WRITE on
#define CONTROL_REGISTER          0x00
#define SERIAL1_REGISTER          0x18
#define SERIAL2_REGISTER          0x19
#define ADDRESS_REGISTER          0x1a
#define PARTY_LINE_REGISTER       0x1e
#define COMMAND_REGISTER          0x40
#define SCALE_VELOCITY_REGISTER   0x45
#define VELOCITY_MODE_REGISTER    0x04
#define OFFSET_REGISTER           0x13

// Values
#define INITIATE_VALUE            0x04
#define PARTY_LINE_ON             0x00
#define PARTY_LINE_OFF            0x08
#define VELOCITY_MODE_DATA        0xa0

// Busyflag from STATUS_REGISTER
#define BUSYFLAG_READY_VALUE      0x00

// Wait between I2C transactions in µs
// One bit every 10µs (2.5µs in 400kHz)
// Wait at least 5 bits to wait for slave answer
#define I2C_WAIT                  50

// Due to I2C problems on the LidarLite v2, it has to be enabled to avoid problems
#define FORCE_RESET_OFFSET        false
#define ENABLE_STRENGTH_MEASURE   false

#define PRINT_DEBUG_INFO          false
#define LIDAR_TIMEOUT_MS          20
#define MAX_LIDARS                8
#define MAX_NACKS                 10

class LidarController {
  public:
    /*******************************************************************************
      begin :
      Start the I2C line with the correct frequency
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
      Add a new Lidar and use resetLidar : It assure the lidar is NOT on the 0x62 line
    *******************************************************************************/
    bool add(LidarObject* _Lidar, uint8_t _id) {
      if (_id >= MAX_LIDARS)
        return false;
      lidars[_id] = _Lidar;
      resetLidar(_id);
      count++;
      return true;
    }

    /*******************************************************************************
      configure : Configure the default acquisition mode

      configuration : The configuration of the Lidar
      
      Lidar : Address of the Lidar (0x62 by default)
    *******************************************************************************/
    void configure(uint8_t Lidar = 0, uint8_t configuration = 2) {
      uint8_t nack = 0;
      switch (configuration){
        case 0: // Default mode, balanced performance
          I2C.write(lidars[Lidar]->address, 0x02,0x80); // Default
          I2C.write(lidars[Lidar]->address, 0x04,0x08); // Default
          I2C.write(lidars[Lidar]->address, 0x1c,0x00); // Default
        break;
    
        case 1: // Short range, high speed
          I2C.write(lidars[Lidar]->address, 0x02,0x1d);
          I2C.write(lidars[Lidar]->address, 0x04,0x08); // Default
          I2C.write(lidars[Lidar]->address, 0x1c,0x00); // Default
        break;
    
        case 2: // Default range, higher speed short range
          I2C.write(lidars[Lidar]->address, 0x02,0x80); // Default
          I2C.write(lidars[Lidar]->address, 0x04,0x00);
          I2C.write(lidars[Lidar]->address, 0x1c,0x00); // Default
        break;
    
        case 3: // Maximum range
          I2C.write(lidars[Lidar]->address, 0x02,0xff);
          I2C.write(lidars[Lidar]->address, 0x04,0x08); // Default
          I2C.write(lidars[Lidar]->address, 0x1c,0x00); // Default
        break;
    
        case 4: // High sensitivity detection, high erroneous measurements
          I2C.write(lidars[Lidar]->address, 0x02,0x80); // Default
          I2C.write(lidars[Lidar]->address, 0x04,0x08); // Default
          I2C.write(lidars[Lidar]->address, 0x1c,0x80);
        break;
    
        case 5: // Low sensitivity detection, low erroneous measurements
          I2C.write(lidars[Lidar]->address, 0x02,0x80); // Default
          I2C.write(lidars[Lidar]->address, 0x04,0x08); // Default
          I2C.write(lidars[Lidar]->address, 0x1c,0xb0);
        break;
        
        /* This mode sometimes kills the laser...
        case 6: // Fastest with errors
          I2C.write(lidars[Lidar]->address, 0x02,0x0d); // Fast acquisition mode from examples
          I2C.write(lidars[Lidar]->address, 0x04,0x04); // Fast acquisition mode from examples
          I2C.write(lidars[Lidar]->address, 0x12,0x03); // Fast acquisition mode from examples
          //I2C.write(lidars[Lidar]->address, 0x1c,0xb0); // Fast acquisition mode from examples
        break*/;
      }
      shouldIncrementNack(Lidar, nack);
    };

    /*******************************************************************************
      changeAddress : Change the address of one Lidar

      returns 0 if success
            1 if error writing the serial number (uint8_t 1)
            2 if error writing the serial number (uint8_t 2)
            3 if error sending the Lidar address
            4 if error disabling the main address
            5 if the new Lidar address is already ON
            6 if the Lidar do not respond
    *******************************************************************************/
    uint8_t changeAddress(uint8_t Lidar) {
      uint8_t _lidar_new = lidars[Lidar]->address;
      uint8_t nack = 0;
      // Return 6 = The device do not respond
      if (!I2C.isOnline(0x62)){
        shouldIncrementNack(Lidar, 1); // If we set anything else than 0, it increments
        return 6;
      }
      // Return 5 = We already got an I2C device at this place
      if (I2C.isOnline(_lidar_new)){
        shouldIncrementNack(Lidar, 1);
        return 5;
      }
      /* Serial number part */
      unsigned char serialNumber[2];
      shouldIncrementNack(Lidar, I2C.readWord(0x62, 0x96, serialNumber));
      // Return 1 = Error sending the Serial (uint8_t 1)
      if (shouldIncrementNack(Lidar, I2C.write(0x62, 0x18, serialNumber[0])))
        return 1;
      // Return 2 = Error sending the Serial (uint8_t 2)
      if (shouldIncrementNack(Lidar, I2C.write(0x62, 0x19, serialNumber[1])))
        return 2;

      // Return 3 = Error sending the Lidar address
      if (shouldIncrementNack(Lidar, I2C.write(0x62, 0x1a, _lidar_new)))
        return 3;

      // Return 4 = Error disabling the Lidar Main address (0x62)
      if (shouldIncrementNack(Lidar, I2C.write(0x62, 0x1e, 0x08)))
        return 4;

      return 0;
    };

    /*******************************************************************************
      status : check the status of the Lidar

      returns the status register (0x01 for version 21 of the Lidar Software)
    *******************************************************************************/
    uint8_t status(uint8_t Lidar = 0) {
      uint8_t data[1] = {171}; // Initializing with a non 0 NOR 1 data to ensure we got
      // no interference
      uint8_t nack = I2C.readByte(lidars[Lidar]->address, 0x01, data);
      shouldIncrementNack(Lidar, nack);
      return data[0];
    };

    /*******************************************************************************
      async : start an acquisition
                  - with preamp enabled
                  - with DC stabilization

      returns the nack error (0 if no error)
    *******************************************************************************/
    uint8_t async(uint8_t Lidar = 0, bool biasCorrection = true) {
      uint8_t nack = 0;
      if(biasCorrection) nack = I2C.write(lidars[Lidar]->address, CONTROL_REGISTER, 0x04);
      else nack = I2C.write(lidars[Lidar]->address, CONTROL_REGISTER, 0x03);
      shouldIncrementNack(Lidar, nack);
    };

    /*******************************************************************************
      distance :
        - Read the measured value from data registers
    *******************************************************************************/
    uint8_t distance(uint8_t Lidar, int * data) {
      uint8_t distanceArray[2];
      uint8_t nackCatcher = I2C.readWord(lidars[Lidar]->address, MEASURED_VALUE_REGISTER, distanceArray);
      shouldIncrementNack(Lidar, nackCatcher);
      int distance = (distanceArray[0] << 8) + distanceArray[1];
      *data = distance;
      return nackCatcher;
    };

    /*******************************************************************************
      Velocity scaling :
        - Scale the velocity measures

        CAUTION, different than the Scale function from LidarLite (not 1,2,3,4 but 
        the actual period measurment, Note the x2 between 100 and 0xC8 (200))

        Measurement  | Velocity         | Register         
        Period (ms)  | Scaling (m/sec)  | 045 Load Value   
        :----------- | :--------------- | :--------------- 
        100          | 0.10 m/s         | 0xC8 (default)   
        40           | 0.25 m/s         | 0x50             
        20           | 0.50 m/s         | 0x28             
        10           | 1.00 m/s         | 0x14             
    *******************************************************************************/
    void scale(uint8_t Lidar, uint8_t velocityScaling){
        I2C.write(lidars[Lidar]->address, SCALE_VELOCITY_REGISTER, velocityScaling);
    };

    /*******************************************************************************
      Velocity  NOT WORKING (guess, since there is no wait) :
        - Read the velocity

        This has to be worked on, this is the original implementation without the 
          blocking architecture
    *******************************************************************************/
    int velocity(uint8_t Lidar, int * data) {
      // Set in velocity mode
      I2C.write(lidars[Lidar]->address, VELOCITY_MODE_REGISTER, VELOCITY_MODE_DATA);
      //  Write 0x04 to register 0x00 to start getting distance readings
      I2C.write(lidars[Lidar]->address, 0x00,0x04);

      uint8_t velocityArray[1];
      uint8_t nack = I2C.readByte(lidars[Lidar]->address, 0x09, velocityArray);

      return((int)((char)velocityArray[0]));
    };

    /*******************************************************************************
      signalStrength :
        - Read the signal strength of the last reading
    *******************************************************************************/
    int signalStrength(uint8_t Lidar, uint8_t * signalStrengthArray) {
      uint8_t nack = I2C.readByte(lidars[Lidar]->address, SIGNAL_STRENGTH_REGISTER, signalStrengthArray);
      shouldIncrementNack(Lidar, nack);
      return nack;
    };

    /*******************************************************************************
      setState : Change the status of the Lidar Object
    *******************************************************************************/
    void setState(uint8_t Lidar = 0, LIDAR_STATE _lidar_state = NEED_RESET) {
      lidars[Lidar]->lidar_state = _lidar_state;
    };

    /*******************************************************************************
      getState : Get the status of the Lidar Object
    *******************************************************************************/
    LIDAR_STATE getState(uint8_t Lidar = 0) {
      return lidars[Lidar]->lidar_state;
    };

    /*******************************************************************************
      setOffset : Set an offset to the Lidar
    *******************************************************************************/
    void setOffset(uint8_t Lidar, uint8_t data) {
        I2C.write(lidars[Lidar]->address, OFFSET_REGISTER, data);
    };

    /*******************************************************************************
      distanceAndAsync : Get the distance then start a new acquisition

      We could use the async() method in ACQUISITION_DONE, but it would need to spin
      one time more before starting the acquisition again
    *******************************************************************************/
    uint8_t distanceAndAsync(uint8_t Lidar, int * data) {
      uint8_t nackCatcher = distance(Lidar, data);
      // if error reading the value, try ONCE again
      if (nackCatcher)
        distance(Lidar, data);
      // Start a new acquisition
      async(Lidar);
      return nackCatcher;
    };

    /*******************************************************************************
      resetLidar :
        * set the Power Enable pin to 0
        * set the Need Reset state to be reinitialized 20ms later
    *******************************************************************************/
    void resetLidar(uint8_t Lidar = 0) {
      lidars[Lidar]->off();
      lidars[Lidar]->timerUpdate();
      setState(Lidar, SHUTING_DOWN);
    };

    /*******************************************************************************
      preReset :
        * set the reset latch (resetOngoing) to true to prevent starting 2 lidars simultaneously
        * set the lidar on & start the 16 µS timer
    *******************************************************************************/
    void preReset(uint8_t Lidar = 0) {
      resetOngoing = true;
      lidars[Lidar]->on();
      lidars[Lidar]->timerUpdate();
    };


    /*******************************************************************************
      getCount :
        * returns the count of the lasers
    *******************************************************************************/
    uint8_t getCount(){
      return count;
    };

    /*******************************************************************************
      postReset :
        * change the lidar address
        * stop the reset ongoing
    *******************************************************************************/
    void postReset(uint8_t Lidar = 0) {
      changeAddress(Lidar);
      resetOngoing = false;
    };


    /*******************************************************************************
      shouldIncrementNack : increments the nacksCount if nack happens
    *******************************************************************************/
    uint8_t shouldIncrementNack(uint8_t Lidar = 0, uint8_t nack = 0){
      if(nack){
        lidars[Lidar]->nacksCount += 1;
      } else {
        // reduce number of resets... but is not valid as it will never reset.
        //lidars[Lidar]->nacksCount = max(lidars[Lidar]->nacksCount - 1, 0); ;
      }
      return nack;
    };

    /*******************************************************************************
      checkNacks : Returns if the laser needs or not a reset
        if have to be resetted, reset the counter and return true. The setState
        instruction have to be in the spinOnce function
    *******************************************************************************/
    bool checkNacks(uint8_t Lidar = 0){
      if(lidars[Lidar]->nacksCount > MAX_NACKS){
        lidars[Lidar]->resetNacksCount();
        return true;
      }
      return false;
    };

    /*******************************************************************************
      spinOnce : Main routine to simplify everything in ASYNC mode, checking the
      status of the Lidar for each loop.
        Cases :
          * ACQUISITION_IN_PROGRESS => Checking the busyFlag
            Get the data and store it in distances
            -> Go to ACQUISITION_READY
          * ACQUISITION_DONE => NOT_USED
          * NEED_RESET => The Lidar is OFF and waits to be started => RESET_PENDING
          * RESET_PENDING => The Lidar is ON, after being OFF and waits 16 µS to be
          ready. No other laser can be on at this time => ACQUISITION_READY
    *******************************************************************************/
    void spinOnce(bool biasCorrection = true) {
      // Handling routine
      //for (int8_t i = count - 1; i >= 0; i--) {
      for(uint8_t i = 0; i<count; i++){
#if PRINT_DEBUG_INFO
        Serial.print("Laser ");
        Serial.print(i);
#endif
        uint8_t strength = 0;
        switch (getState(i)) {
          
          case ACQUISITION_IN_PROGRESS : 
#if PRINT_DEBUG_INFO
            Serial.println(" ACQUISITION_IN_PROGRESS ");
#endif
            // Get the status bit, if 0 => Acquisition is done
            if (bitRead( status(i), 0) == 0) {
              async(i, biasCorrection); // launch next measure before reading our measure. 
              
              lidars[i]->last_distance = lidars[i]->distance;            
              distance(i, &lidars[i]->distance);
#if ENABLE_STRENGTH_MEASURE
              signalStrength(i, &lidars[i]->strength);
#endif
#if PRINT_DEBUG_INFO
              Serial.println(i);
              Serial.println(lidars[i]->distance);
#endif
              if((abs(lidars[i]->distance - lidars[i]->last_distance) > 100) | (lidars[i]->distance < 4 or lidars[i]->distance > 1000)){
                shouldIncrementNack(i, 1);
              }
              
              lidars[i]->notify_distance();
#if FORCE_RESET_OFFSET
              setOffset(i, 0x00);
#endif
              lidars[i]->lastMeasure = micros();
            } else {
              if(lidars[i]->checkLastMeasure()){
                setState(i, SHUTING_DOWN);
              }
            }
            break;
            
          case NEED_RESET:
#if PRINT_DEBUG_INFO
            Serial.println(" NEED_RESET");
#endif
            if (!resetOngoing) {
              preReset(i);
              setState(i, RESET_PENDING);
            }
            break;
          case RESET_PENDING:
#if PRINT_DEBUG_INFO
            Serial.println(" RESET_PENDING");
#endif
            // Check the timer, if done, laser is ready to reset, change state
            if (lidars[i]->checkTimer()) {
              postReset(i);
              configure(i, lidars[i]->configuration);
              async(i);
              lidars[i]->lastMeasure = micros();
              setState(i, ACQUISITION_IN_PROGRESS);
            }
            break;

          case SHUTING_DOWN :
#if PRINT_DEBUG_INFO
            Serial.println(" SHUTING_DOWN");
#endif
            if (lidars[i]->checkTimer()) {
              setState(i, NEED_RESET);
            }
            break;
          default:
            break;
        } // End switch case

        if(checkNacks(i)){
           //resetLidar(i);
        }
      } // End for each laser
    };

    LidarObject* lidars[MAX_LIDARS];
  private:
    bool resetOngoing = false;
    uint8_t count = 0;
};


#endif

