#include <TXPacket.h>
#include <RXPacket.h>
#include <PacketController.h>

PacketController controller;
PacketStatus pStatus;

void setup() {
  controller.begin();
}

void loop() {
  /*
  Serial.println("Writing Data");
  
  Serial1.write(0xAB); //JUNK
  
  Serial1.write(0xBD); //HEADER
  Serial1.write(0xFA);
  
  Serial1.write((uint8_t)0); //VEL
  Serial1.write((int8_t)1);
  Serial1.write((int8_t)2);
  
  Serial1.write((int8_t)-1); //ROT
  Serial1.write((int8_t)11);
  Serial1.write((int8_t)-2);
  
  Serial1.write((int8_t)1); //POSZ
  Serial1.write((int8_t)1);
  
  Serial1.write((int8_t)1); //TOR
  
  Serial1.write((int8_t)1); //SERVO
  Serial1.write((int8_t)1);
  Serial1.write((int8_t)1);
  Serial1.write((int8_t)1);
  Serial1.write((int8_t)1);
  Serial1.write((int8_t)1);
  
  Serial1.write((int8_t)1); //SPARE
  
  Serial1.write(0xBF); //CHECKSUM
  Serial1.write(0x0E);
  
  delay(1000);
  */
  Serial.println("Listening");
  pStatus = controller.listen();
  
  delay(1000);
  
  if (pStatus == VALID_PACKET) {
    Serial.println("VALID PACKET");
    Serial.print("VELX: "); Serial.println(controller.get(VELX));
    Serial.print("VELY: "); Serial.println(controller.get(VELY));
    Serial.print("VELZ: "); Serial.println(controller.get(VELZ));
    Serial.print("ROTX: "); Serial.println(controller.get(ROTX));
    Serial.print("ROTY: "); Serial.println(controller.get(ROTY));
    Serial.print("ROTZ: "); Serial.println(controller.get(ROTZ));
    Serial.print("POSZ: "); Serial.println((int16_t)controller.getPosZ());
    
  } else if (pStatus == NO_HEADER_FOUND) {
    Serial.println("NO HEADER FOUND");
  } else if (pStatus == INVALID_PACKET) {
   Serial.println("INVALID PACKET");
   Serial.print("VELX: "); Serial.println((int8_t)controller.get(VELX));
   Serial.print("VELY: "); Serial.println(controller.get(VELY));
   Serial.print("VELZ: "); Serial.println(controller.get(VELZ));
   Serial.print("ROTX: "); Serial.println(controller.get(ROTX));
   Serial.print("ROTY: "); Serial.println(controller.get(ROTY));
   Serial.print("ROTZ: "); Serial.println(controller.get(ROTZ));
   Serial.print("POSZ: "); Serial.println((int16_t)controller.getPosZ());
   
  } else if (pStatus == NOT_ENOUGH_DATA) {
    Serial.println("NOT ENOUGH DATA");
  }
  
  delay(1000);
}


