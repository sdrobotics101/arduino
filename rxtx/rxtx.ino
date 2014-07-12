#include <TXPacket.h>
#include <RXPacket.h>
#include <PacketController.h>

PacketController controller(Serial3, Serial3);

int8_t val = 0;
//bool pin_2_val = LOW;
void setup() {
    controller.begin();
  //  pinMode(2, OUTPUT);
  //  digitalWrite(2, pin_2_val);
}

void loop() {
  
  //pin_2_val = !pin_2_val;
  //digitalWrite(2, pin_2_val);
  //Serial.println(pin_2_val ? "1" : "0");
  //digitalWrite(2, HIGH);
  
    //Serial.println(val);
    while (controller.listen() != VALID_PACKET) {
        ;
    }
    
    //Serial.println("  Received Packet");
    //Serial.println("");
    //Serial.print("    VELX: "); 
      Serial.println(controller.get(VELX));
    //Serial.print("    VELY: "); Serial.println(controller.get(VELY));
    //Serial.print("    VELZ: "); Serial.println(controller.get(VELZ));
    //Serial.print("    ROTX: "); Serial.println(controller.get(ROTX));
    //Serial.print("    ROTY: "); Serial.println(controller.get(ROTY));
    //Serial.print("    ROTZ: "); Serial.println(controller.get(ROTZ));
    //Serial.print("    POSZ: "); Serial.println((int16_t)controller.getPosZ());
    //Serial.println("");
    
    //Serial.println("      Assigning Values");
    controller.set(ACCX, val);
    controller.set(ACCY, val+1);
    controller.set(ACCZ, val+2);
    controller.set(MAGX, val+3);
    controller.set(MAGY, val+4);
    controller.set(MAGZ, val+5);
    controller.set(PRESSURE, val+6);
    controller.set(TXSPARE, val+7);
    //Serial.println("      Assigned Values");
    //Serial.println("    Sending");
    controller.send();
    //Serial.println("    Sent");
    val++;
  //digitalWrite(2, LOW);
}
