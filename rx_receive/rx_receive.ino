#include <Wire.h>
#include <Cubeception.h>

PacketController controller;
PacketStatus pStatus;

void setup() {
  Serial.begin(115200);
  controller.begin();
}

void loop() {
  /*
  Serial.println("Writing Data");
  
  Serial1.write(0xAB); //JUNK
  
  Serial1.write(0xFA); //HEADER
  Serial1.write(0xBD);
  
  Serial1.write((uint8_t)0); //VEL
  Serial1.write((int8_t)1);
  Serial1.write((int8_t)2);
  
  Serial1.write((int8_t)-1); //ROT
  
  Serial1.write((uint8_t)1); //TOR
  
  Serial1.write((uint8_t)1); //SERVO

  Serial1.write((uint8_t)1); //LED
  
  Serial1.write((uint8_t)1); //MODE
  Serial1.write((uint8_t)1);
    
  Serial1.write(0xBE); //CHECKSUM
  Serial1.write(0x02);
  
  delay(1000);
  */
  //Serial.println("Listening");
  pStatus = controller.listen();
  
  //delay(1000);
  
  if (pStatus == VALID_PACKET) {
    Serial.println("VALID PACKET");
    Serial.print("VELX: "); Serial.println(controller.getS8(VELX));
    Serial.print("VELY: "); Serial.println(controller.getS8(VELY));
    Serial.print("VELZ: "); Serial.println(controller.getS8(VELZ));
    Serial.print("ROTZ: "); Serial.println(controller.getS8(ROTZ));
    Serial.print("TORP: "); Serial.println(controller.getU8(TORPEDOCTL));
    Serial.print("SERV: "); Serial.println(controller.getU8(SERVOCTL));
    Serial.print("LED : "); Serial.println(controller.getU8(LEDCTL));
    Serial.print("MODE: "); Serial.println(controller.getU16(MODE));
    
  } else if (pStatus == NO_HEADER_FOUND) {
    Serial.println("NO HEADER FOUND");
  } else if (pStatus == INVALID_PACKET) {
   Serial.println("INVALID PACKET");
   Serial.print("VELX: "); Serial.println(controller.getS8(VELX));
   Serial.print("VELY: "); Serial.println(controller.getS8(VELY));
   Serial.print("VELZ: "); Serial.println(controller.getS8(VELZ));
   Serial.print("ROTZ: "); Serial.println(controller.getS8(ROTZ));
   Serial.print("TORP: "); Serial.println(controller.getU8(TORPEDOCTL));
   Serial.print("SERV: "); Serial.println(controller.getU8(SERVOCTL));
   Serial.print("LED : "); Serial.println(controller.getU8(LEDCTL));
   Serial.print("MODE: "); Serial.println(controller.getU16(MODE)); 
   
  } else if (pStatus == NOT_ENOUGH_DATA) {
    Serial.println("NOT ENOUGH DATA");
  }
  
  delay(50);
}


