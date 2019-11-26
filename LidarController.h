#ifndef LIDAR_CONTROLLER_H
#define LIDAR_CONTROLLER_H

#include "I2CFunctions.h"
#include "LidarObject.h"
// Wait between I2C transactions in µs
// One bit every 10µs (2.5µs in 400kHz)
// Wait at least 5 bits to wait for slave answer
// Not used
#define I2C_WAIT                  50

// Due to I2C problems on the LidarLite v2, it has to be enabled to avoid problems
#define FORCE_RESET_OFFSET        true
#define ENABLE_STRENGTH_MEASURE   true

// For debug purpose
#define PRINT_DEBUG_INFO          false
#define MAX_LIDARS                8
// Force reset on errors. It resets the lasers on MAX_NACKS error.
// An error is a misreading (nack) or an incoherent value
#define MAX_NACKS                 25
#define ERROR_MAX_VALUE           1000
#define ERROR_MIN_VALUE           4
// Difference of measure between two measurements
#define ERROR_MAX_DIFF_VALUE      100


// Registers are separated between READ & WRITE registers.
// Indeed the result reading or writing to the same internal register does not affect
// the Lidar the same way
// All registers are not used here, but they are ready to be used in newer versions

// READ Registers
// Those are registers we only READ from
// System status
/** 
 * bit 6: Process error flag
 * - 0 No error
 * - 1 Error
 * bit 5: Health flag
 * - 0 Error
 * - 1 No error
 * bit 4: Secondary return flag
 * - 0 No secondary return
 * - 1 Secondary return
 * bit 3: Invalid signal flag
 * - 0 Peak detected
 * - 1 No peak detected
 * bit 2: Signal overflow flag
 * - 0 Overflow detected
 * - 1 No overflow
 * bit 1: Reference overflow flag
 * - 0 Overflow detected
 * - 1 No overflow
 * bit 0: Busy flag
 * - 0 Ready 
 * - 1 Busy
 */
#define REG_STATUS                0x01 
// Velocity output  
#define REG_VELOCITY              0x09
// Correlation Record noise floor
#define REG_NOISE_PEAK            0x0d
// Signal strength
#define REG_SIGNAL_STRENGTH       0x0e
// Distance measurement delay
#define REG_FULL_DELAY_HIGH       0x0f
#define REG_FULL_DELAY_LOW        0x10
// Previous distance measurement
#define LAST_DELAY_HIGH           0x14
#define LAST_DELAY_LOW            0x15
// Unit serial unique code
#define REG_UNIT_ID_HIGH          0x16
#define REG_UNIT_ID_LOW           0x17
// Second largest peak value in correlation record
#define REG_PEAK_BCK              0x4c
// Correlation record data low & high byte
#define REG_CORR_DATA_LOW         0x52
#define REG_CORR_DATA_HIGH        0x53
#define REG_CORR_DATA_DUAL        0xd2

#define MEASURED_VALUE_REGISTER   0x8f
#define READ_SERIAL_REGISTERS     0x96

// WRITE Registers
// Those are register we only WRITE on
// COMMAND REGISTER
#define REG_ACQ_COMMAND           0x00
// Maximum acquisition count, default 0x80
#define REG_SIG_CONT_VAL          0x02
// Acquisition mode control, default 0x08
/** 
 * bit 6:
 * - 0 Enable reference process during measurement
 * - 1 Disable reference process during measurement
 * bit 5:
 * - 0 Use default delay for burst and free running mode
 * - 1 Use REG_MEASURE_DELAY for burst and free running mode
 * bit 4:
 * - 0 Enable reference filter (average 8 measurements)
 * - 1 Disable reference filter
 * bit 3:
 * - 0 Enable quick termination if maximal delay reached
 * - 1 Disable quick termination if maximal delay reached
 * bit 2:
 * - 0 Use default acquisition count of 5
 * - 1 Use REG_REF_COUNT_VAL acquisition count.
 * bit 0-1:
 * - 00 default pwm mode, pull trigger low to trigger, high output (mode) of of 10us/cm
 * - 01 Status output mode, mode will be put high while busy and low when done
 * - 10 Fixed delay PWM, pulling trig to low does not trigger a measurement
 * - 11 Oscillator output mode, output 31.25kHz +/-1% 
 */
#define REG_ACQ_CONFIG            0x04
// Burst measurement count control, default 0x08
/**
 * - 0x00-0x01 One measurement per command
 * - 0x02-0xfe Repetition count
 * - 0xff Indefinite repetition after initial command
 */
#define REG_OUTER_LOOP_COUNT      0x11
// Reference acquisition count, default 0x05
#define REG_REF_COUNT_VAL         0x12
// Offset register (signed int8_t), default 0x00 (disabled in v3?)
#define REG_OFFSET_REGISTER       0x13
// Write ID to this register for I2C address change unlock
#define REG_I2C_ID_HIGH           0x18
#define REG_I2C_ID_LOW            0x19
// I2C address register
#define REG_I2C_SEC_ADDR          0x1a
// Peak detection threshold bypass, default 0x00
/**
 * - 0x00 Default valid measurement
 * - 0x01-0xff Set threshold value for valid measurement (0x20 - 0x60 performs well)
 * - 0xff Indefinite repetition after initial command
 */
#define REG_THRESHOLD_BYPASS      0x1c
// Default address response control, default 0x00 = party line on, 0x08 = party line off, the bit 8
#define REG_I2C_CONFIG            0x1e
// State command
// Read is error register
/**
 * - 0x00 Disable test mode
 * - 0x07 Enable test mode, ability to read correlation data
 * - C
 */
#define REG_COMMAND               0x40
// Delay between measurements (and velocity scale register), default 0x14
#define REG_MEASURE_DELAY         0x45
// Correlation record memory bank select
#define REG_ACQ_SETTINGS          0x5d
// Power state control, default 0x80
#define REG_POWER_CONTROL         0x65

// Typical register data to write
#define DATA_RESET_ALL            0x00
#define DATA_MEASURE_WITHOUT_BIAS 0x03
#define DATA_MEASURE_WITH_BIAS    0x04
#define DATA_PARTY_LINE_ON        0x00
#define DATA_PARTY_LINE_OFF       0x08
#define DATA_VELOCITY_MODE_DATA   0xa0

class LidarController {
  public:
    /*******************************************************************************
      begin:
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
      add:
      Add a new Lidar and use resetLidar: It assure the lidar is NOT on the 0x62 line
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
      configure: Configure the default acquisition mode

      configuration: The configuration of the Lidar
      
      Lidar: Address of the Lidar (0x62 by default)
    *******************************************************************************/
    void configure(uint8_t Lidar = 0, uint8_t configuration = 2) {
      // REG_SIG_CONT_VAL = Maximum acquisition count, default 0x80
      // REG_ACQ_CONFIG = Acquisition mode control, default 0x08
      // REG_THRESHOLD_BYPASS = Peak detection threshold bypass, default 0x00
      switch (configuration){
        case 0: // Default mode, balanced performance
          I2C.write(lidars[Lidar]->address, REG_SIG_CONT_VAL, 0x80); // Default
          I2C.write(lidars[Lidar]->address, REG_ACQ_CONFIG, 0x08); // Default
          I2C.write(lidars[Lidar]->address, REG_THRESHOLD_BYPASS, 0x00); // Default
        break;
    
        case 1: // Short range, high speed
          I2C.write(lidars[Lidar]->address, REG_SIG_CONT_VAL, 0x1d);
          I2C.write(lidars[Lidar]->address, REG_ACQ_CONFIG, 0x08); // Default
          I2C.write(lidars[Lidar]->address, REG_THRESHOLD_BYPASS, 0x00); // Default
        break;
    
        case 2: // Default range, higher speed short range
          I2C.write(lidars[Lidar]->address, REG_SIG_CONT_VAL, 0x80); // Default
          I2C.write(lidars[Lidar]->address, REG_ACQ_CONFIG, 0x00);
          I2C.write(lidars[Lidar]->address, REG_THRESHOLD_BYPASS, 0x00); // Default
        break;
    
        case 3: // Maximum range
          I2C.write(lidars[Lidar]->address, REG_SIG_CONT_VAL, 0xff);
          I2C.write(lidars[Lidar]->address, REG_ACQ_CONFIG, 0x08); // Default
          I2C.write(lidars[Lidar]->address, REG_THRESHOLD_BYPASS, 0x00); // Default
        break;
    
        case 4: // High sensitivity detection, high erroneous measurements
          I2C.write(lidars[Lidar]->address, REG_SIG_CONT_VAL, 0x80); // Default
          I2C.write(lidars[Lidar]->address, REG_ACQ_CONFIG, 0x08); // Default
          I2C.write(lidars[Lidar]->address, REG_THRESHOLD_BYPASS, 0x80);
        break;
    
        case 5: // Low sensitivity detection, low erroneous measurements
          I2C.write(lidars[Lidar]->address, REG_SIG_CONT_VAL, 0x80); // Default
          I2C.write(lidars[Lidar]->address, REG_ACQ_CONFIG, 0x08); // Default
          I2C.write(lidars[Lidar]->address, REG_THRESHOLD_BYPASS, 0xb0);
        break;
      }
    };

    /*******************************************************************************
      changeAddress: Change the address of one Lidar

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
      status: check the status of the Lidar

      returns the status register (0x01 for version 21 of the Lidar Software)
    *******************************************************************************/
    uint8_t status(uint8_t Lidar = 0) {
      uint8_t data[1] = {171}; // Initializing with a non 0 NOR 1 data to ensure we got
      // no interference
      uint8_t nack = I2C.readByte(lidars[Lidar]->address, REG_STATUS, data);
      shouldIncrementNack(Lidar, nack);
      return data[0];
    };

    /*******************************************************************************
      async: start an acquisition
                  - with preamp enabled
                  - with DC stabilization

      returns the nack error (0 if no error)
    *******************************************************************************/
    uint8_t async(uint8_t Lidar = 0, bool biasCorrection = true) {
      uint8_t nack = 0;
      if(biasCorrection) nack = I2C.write(lidars[Lidar]->address, REG_ACQ_COMMAND, DATA_MEASURE_WITH_BIAS);
      else nack = I2C.write(lidars[Lidar]->address, REG_ACQ_COMMAND, DATA_MEASURE_WITHOUT_BIAS);
      shouldIncrementNack(Lidar, nack);
	  return nack;
    };

    /*******************************************************************************
      distance:
        - Read the measured value from data registers
    *******************************************************************************/
    //uint8_t distance(uint8_t Lidar, int * data) {
	uint8_t distance(uint8_t Lidar, int16_t * data) {
      uint8_t distanceArray[2];
      uint8_t nackCatcher = I2C.readWord(lidars[Lidar]->address, MEASURED_VALUE_REGISTER, distanceArray);
      shouldIncrementNack(Lidar, nackCatcher);
      int16_t distance = (distanceArray[0] << 8) + distanceArray[1];
      *data = distance;
      return nackCatcher;
    };

    /*******************************************************************************
      Velocity scaling:
        - Scale the velocity measures

        CAUTION, different than the Scale function from LidarLite (not 1,2,3,4 but 
        the actual period measurement, Note the x2 between 100 and 0xC8 (200))

        Measurement  | Velocity         | Register         
        Period (ms)  | Scaling (m/sec)  | 045 Load Value   
       :----------- |:--------------- |:--------------- 
        100          | 0.10 m/s         | 0xC8 (default)   
        40           | 0.25 m/s         | 0x50             
        20           | 0.50 m/s         | 0x28             
        10           | 1.00 m/s         | 0x14             
    *******************************************************************************/
    void scale(uint8_t Lidar, uint8_t velocityScaling){
        I2C.write(lidars[Lidar]->address, REG_MEASURE_DELAY, velocityScaling);
    };

    /*******************************************************************************
      Velocity  NOT WORKING (guess, since there is no wait):
        - Read the velocity

        This has to be worked on, this is the original implementation without the 
          blocking architecture
    *******************************************************************************/
    int16_t velocity(uint8_t Lidar, int16_t * data) {
      // Set in velocity mode
      I2C.write(lidars[Lidar]->address, REG_ACQ_CONFIG, DATA_VELOCITY_MODE_DATA);
      //  Write 0x04 to register 0x00 to start getting distance readings
      I2C.write(lidars[Lidar]->address, REG_ACQ_COMMAND, DATA_MEASURE_WITH_BIAS);

      uint8_t velocityArray[1];
      uint8_t nack = I2C.readByte(lidars[Lidar]->address, REG_VELOCITY, velocityArray);

      return((int16_t)((char)velocityArray[0]));
    };

    /*******************************************************************************
      signalStrength:
        - Read the signal strength of the last reading
    *******************************************************************************/
    uint8_t signalStrength(uint8_t Lidar, uint8_t * signalStrengthArray) {
      uint8_t nack = I2C.readByte(lidars[Lidar]->address, REG_SIGNAL_STRENGTH, signalStrengthArray);
      shouldIncrementNack(Lidar, nack);
      return nack;
    };

    /*******************************************************************************
      setState: Change the status of the Lidar Object
    *******************************************************************************/
    void setState(uint8_t Lidar = 0, LIDAR_STATE _lidar_state = NEED_RESET) {
      lidars[Lidar]->lidar_state = _lidar_state;
    };

    /*******************************************************************************
      getState: Get the status of the Lidar Object
    *******************************************************************************/
    LIDAR_STATE getState(uint8_t Lidar = 0) {
      return lidars[Lidar]->lidar_state;
    };

    /*******************************************************************************
      setOffset: Set an offset to the Lidar
    *******************************************************************************/
    void setOffset(uint8_t Lidar, uint8_t data) {
        I2C.write(lidars[Lidar]->address, REG_OFFSET_REGISTER, data);
    };

    /*******************************************************************************
      distanceAndAsync: Get the distance then start a new acquisition

      We could use the async() method in ACQUISITION_DONE, but it would need to spin
      one time more before starting the acquisition again
    *******************************************************************************/
    uint8_t distanceAndAsync(uint8_t Lidar, int16_t * data) {
      uint8_t nackCatcher = distance(Lidar, data);
      // if error reading the value, try ONCE again
      if (nackCatcher)
        distance(Lidar, data);
      // Start a new acquisition
      async(Lidar);
      return nackCatcher;
    };

    /*******************************************************************************
      resetLidar:
        * set the Power Enable pin to 0
        * set the Need Reset state to be reinitialized 20ms later
    *******************************************************************************/
    void resetLidar(uint8_t Lidar = 0) {
      lidars[Lidar]->off();
      lidars[Lidar]->timerUpdate();
      setState(Lidar, SHUTING_DOWN);
    };

    /*******************************************************************************
      preReset:
        * set the reset latch (resetOngoing) to true to prevent starting 2 lidars simultaneously
        * set the lidar on & start the 16 µS timer
    *******************************************************************************/
    void preReset(uint8_t Lidar = 0) {
      resetOngoing = true;
      lidars[Lidar]->on();
      lidars[Lidar]->timerUpdate();
    };


    /*******************************************************************************
      getCount:
        * returns the count of the lasers
    *******************************************************************************/
    uint8_t getCount(){
      return count;
    };

    /*******************************************************************************
      postReset:
        * change the lidar address
        * stop the reset ongoing
    *******************************************************************************/
    void postReset(uint8_t Lidar = 0) {
      changeAddress(Lidar);
      resetOngoing = false;
    };


    /*******************************************************************************
      shouldIncrementNack: increments the nacksCount if nack happens
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
      checkNacks: Returns if the laser needs or not a reset
        if have to be reset, reset the counter and return true. The setState
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
      spinOnce: Main routine to simplify everything in ASYNC mode, checking the
      status of the Lidar for each loop.
        Cases:
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
      for(uint8_t i = 0; i < count; i++){
#if PRINT_DEBUG_INFO
        Serial.print("Laser ");
        Serial.print(i);
#endif
        uint8_t strength = 0;
        switch (getState(i)) {
          
          case ACQUISITION_IN_PROGRESS: 
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
              if((abs(lidars[i]->distance - lidars[i]->last_distance) > ERROR_MAX_DIFF_VALUE) | (lidars[i]->distance < ERROR_MIN_VALUE or lidars[i]->distance > ERROR_MAX_VALUE)){
                shouldIncrementNack(i, 1);
              }
              
              lidars[i]->notify_distance();
#if FORCE_RESET_OFFSET
              setOffset(i, 0x00);
#endif
              lidars[i]->lastMeasureTime = micros();
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
              lidars[i]->lastMeasureTime = micros();
              setState(i, ACQUISITION_IN_PROGRESS);
            }
            break;

          case SHUTING_DOWN:
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
