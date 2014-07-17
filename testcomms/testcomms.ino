#include <TXPacket.h>
#include <RXPacket.h>
#include <PacketController.h>

PacketController packetController;
PacketStatus pStatus = NO_HEADER_FOUND;

int8_t velX;
int8_t velY;
int8_t velZ;
int8_t rotX;
int8_t rotY;
int8_t rotZ;
int8_t torpedo;
int8_t servo;
int8_t servo1;
int8_t servo2;
int8_t servo3;
int8_t servo4;
int8_t servo5;
int16_t posZ;
int8_t spare;

void setup() {
  packetController.begin();
}

void loop() {
  pStatus = packetController.listen();
  
  velX = packetController.get8(VELX);
  velY = packetController.get8(VELY);
  velZ = packetController.get8(VELZ);
  posZ = packetController.get16(POSZ);
  
  if (pStatus == VALID_PACKET) {
    Serial.println("VALID PACKET");
    Serial.print(velX); Serial.print(" ");
    Serial.print(velY); Serial.print(" ");
    Serial.print(velZ); Serial.print(" ");
    Serial.print(posZ); Serial.print(" ");
    Serial.println("");
  } else if (pStatus == INVALID_PACKET) {
    Serial.println("INVALID PACKET");
    Serial.print(velX); Serial.print(" ");
    Serial.print(velY); Serial.print(" ");
    Serial.print(velZ); Serial.print(" ");
    Serial.print(posZ); Serial.print(" ");
    Serial.println(""); 
  } 
  delay(2);
 }
