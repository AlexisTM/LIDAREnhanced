/*
 * DATA Structure
 *
 * N-packet (1 byte)
 * N-int16_t (1 byte)
 * data 0-123(int) // MAX_MEASURES
 * checksum(1 byte)
 * 
 */


#ifndef PACKET_COMM_H
#define PACKET_COMM_H

#define MAX_MEASURES 6
#define INT_SIZE 2
#define MEASURES_SIZE MAX_MEASURES*INT_SIZE
#define MESSAGE_SIZE MEASURES_SIZE+3
#define CHECKSUM_SIZE MESSAGE_SIZE-1

class PacketComm
{
public:
  PacketComm(HardwareSerial * _serial){
    serial = _serial;
  };

  ~PacketComm(){};

  void addData(int16_t * data, uint8_t index){
    memcpy(&Message[2+index*INT_SIZE], data, INT_SIZE);
  };

  void reset(){
    memset(Message,0,MESSAGE_SIZE);
  };

  void packMessage(){
    Message[0] = count;
    Message[1] = MAX_MEASURES;
    Message[CHECKSUM_SIZE] = doChecksum();
  };


  uint8_t doChecksum(){
    uint8_t data = 0;
    for(byte i = 0; i < CHECKSUM_SIZE ; i++){
      data += Message[i];
    }
    return data;
  };
  uint8_t getMessage(byte * address){
    address = Message;
    return count;
  };

  void sendMessage(){
    packMessage();
    int sent = serial->write((uint8_t*) Message, MESSAGE_SIZE);
    count++;
  }

private:
  HardwareSerial* serial;
  uint8_t count = 0;
  uint8_t Message[MESSAGE_SIZE];
  //int16_t Measures[MAX_MEASURES];
};

#endif
