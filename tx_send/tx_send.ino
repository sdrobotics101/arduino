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
    
    controller.set(ACCX, 0);
    controller.set(ACCY, -1);
    controller.set(ACCZ, 2);
    controller.set(MAGX, 3);
    controller.set(MAGY, 4);
    controller.set(MAGZ, 5);
    controller.set(PRESSURE, 6);
    controller.set(TXSPARE, 7);
    
    controller.send();
    Serial.println("Packet Sent");
    
    delay(3000);
    
    Serial.println("Begin Packet");
    while (Serial1.available() > 0) {
        Serial.print((int8_t)Serial1.read());
        Serial.print(" ");
    }
    Serial.println("");
    Serial.println("End Packet");
    
    delay(100);
    
}
