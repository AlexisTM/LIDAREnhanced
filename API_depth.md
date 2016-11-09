API in depth
===============

### Lidar object
The lidar object represents a laser.

#### LidarObject::begin

```C++
void LidarObject.begin(EnablePin, ModePin, I2CAddress, LaserConfiguration, LidarMode, OneCharName);

// used as 
static LidarObject LZ1;
LZ1.begin(12, 13, 0x64, 2, DISTANCE, 'x');
```

#### LIDAR_STATE enum

```C++
enum LIDAR_STATE {
  SHUTING_DOWN = 240,       // Shutdown the laser to reset it
  NEED_RESET = 48,          // Too much outliers, need to reset
  RESET_PENDING = 80,       // Wait 15ms after you reset the Lidar, we are waiting in this state
  NEED_CONFIGURE = 144,     // 15ms passed, we now configure the Lidar
  ACQUISITION_READY = 32,   // I started an acquisition, need someone to read it
  ACQUISITION_PENDING = 64, // The acquisition in on progress
  ACQUISITION_DONE = 128    // I read the data, need to start an acq again
};
```

#### LIDAR_MODE enum

```C++
enum LIDAR_MODE {
  NONE = 0,                  // Should not be used unless you want to disable a connected laser (could be implemented)
  DISTANCE = 1,              // Measure distance as fast as possible
  VELOCITY = 2,              // Measure velocity at a certain rate
  DISTANCE_AND_VELOCITY = 3  // Measure distance and velocity
};
```

#### Data reading

```C++ 
    int distance;      // newest measure
    int last_distance; // last measure
    int velocity;       // newest velocity
    uint8_t strength;   // newest signal strength
```

### LidarController object

#### LidarController::begin
Start the I2C line with or without fasti2c (400kHz)
```C++
void begin(bool fasti2c = false);
```

#### LidarController::add
Add a lidar to the controller, if the id is over the maximum number of lidar (8 by default), returns false and do not add the lidar.

```C++
bool add(LidarObject* _Lidar, uint8_t _id);
```

#### LidarController::spinOnce

Make a new step in the Lidar State Machine, store results to *int LidarController::distances*

The functions below are NOT needed if you do not want to use the state machine

```C++
void spinOnce();
```

#### LidarController::configure

Send the Lidar configuration to the Lidar (id of the lidar)
  - 0 = basic configuration
  - 1 = faster (Do not read 3 times), bit noisier
  - 2 = low noise, low sensitivity, less false detection (Default)
  - 3 = High noise, high sensitivity

```C++
configure(uint8_t Lidar = 0, uint8_t configuration = 2);
```

#### LidarController::changeAddress

Change the address of the Lidar to the address configured in the object
```C++
changeAddress(uint8_t Lidar);
```

#### LidarController::status

Returns the status uint8_t of the Lidar (id)

```C++
uint8_t status(uint8_t Lidar = 0);
```
#### LidarController::async

Send the command to the lidar (id) to start an acquisition

```C++
uint8_t async(uint8_t Lidar = 0);
```

#### LidarController::distance

Read the distance from the Lidar (id)

NOTE : You have to send the command async FIRST and should check if the data is there. You can know it by reading the bit 0 from the status of the lidar (LidarController::status)

```C++
int distance(uint8_t Lidar, int * data);
```

#### LidarController::distanceAndAsync

Read the distance from the Lidar (id) and restart an acquisition, writes to data and returns nack status

```C++
uint8_t distanceAndAsync(uint8_t Lidar, int * data);
```

#### LidarController::resetLidar

Reset a Lidar

```C++
void resetLidar(uint8_t Lidar = 0)
```


### I2C object

#### I2C::begin
Start the I2C line
```C++
void I2C.begin(fasti2c = false);
```

#### I2C::isOnline
Check if the device/laser is online 
```C++
void I2C.isOnline(Device = 0x62);
```

#### I2C::write
Write data though I2C to a device
```C++
uint8_t write(uint8_t Device, uint8_t regAdr, uint8_t data);
```

#### I2C::readByte 
Read one uint8_t from an I2C device, stored in data, returns NACK status
```C++
uint8_t readByte(uint8_t Device, uint8_t regAdr, uint8_t * data)
```

#### I2C::readWord
Read *two* uint8_t from an I2C device, stored in data, returns NACK status
```C++
uint8_t readWord(uint8_t Device, uint8_t regAdr, uint8_t * data)
```
