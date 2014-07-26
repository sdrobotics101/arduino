#include <Wire.h>
#include <SPI.h>
#include <Cubeception.h>

PacketController controller(RX_SERIAL_PORT, TX_SERIAL_PORT);

void setup() {
    Serial.begin(115200);
    controller.begin();
}

void loop() {
    Serial.println("Loop Start");
    delay(100);
    Serial.println("Setting Variables");
   
    controller.setS16(MAGX, 10000);
    controller.setS16(MAGY, -135);
    controller.setS16(MAGZ, -15000);
    controller.setU16(POSZ, 10000);
    controller.setU16(HEALTH, 0);
    controller.setU8(BATV, 255);
    
    delay(100);
    Serial.println("Sending Packet");
    
    controller.send();
    Serial.println("Packet Sent");
    /*
    Serial.println("Loop End");
    delay(3000);
    
    Serial.println("Begin Packet");
    while (Serial1.available() > 0) {
        Serial.print(Serial1.read(), HEX);
        Serial.print(" ");
    }
    Serial.println("");
    Serial.println("End Packet");
    
    delay(100);
    */
}
