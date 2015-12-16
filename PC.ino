#include "PacketComm.h"

static PacketComm CommunicationPi = PacketComm(&Serial);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  for(byte i = 0; i < 6; i++){
    int data = 10*i+10;
    int16_t data16 = int16_t(data);
    CommunicationPi.addData(&data16, i);
  }
  CommunicationPi.sendMessage();
  CommunicationPi.reset();
    
}
