#include <TXPacket.h>
#include <RXPacket.h>
#include <PacketController.h>

PacketController controller;

void setup() {
    controller.begin();
}

void loop() {
    Serial.println("Loop Start");
    delay(100);
    Serial.println("Setting Variables");
    controller.set(ACCX, 1);
    controller.set(ACCY, 2);
    controller.set(ACCZ, 3);
    controller.set(MAGX, 4);
    controller.set(MAGY, 5);
    controller.set(MAGZ, 6);
    controller.set(PRESSURE, 7);
    controller.set(TXSPARE, 8);
    
    delay(100);
    Serial.println("Sending Packet");
    
    controller.send();
    Serial.println("Packet Sent");
    
    Serial.println("Loop End");
    delay(3000);
    /*
    Serial.println("Begin Packet");
    while (Serial1.available() > 0) {
        Serial.print((int8_t)Serial1.read());
        Serial.print(" ");
    }
    Serial.println("");
    Serial.println("End Packet");
    
    delay(100);
    */
}
