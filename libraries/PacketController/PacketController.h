//
//  PacketController.h
//  
//
//  Created by Rahul Salvi on 6/28/14.
//
//

#ifndef ____PacketController__
#define ____PacketController__

#include "Arduino.h"
#include "RXPacket.h"
#include "TXPacket.h"

class PacketController {
public:
    PacketController(USARTClass &rxSerialPort = Serial3, USARTClass &txSerialPort = Serial3);
    void begin(int baudRate = 115200);
    
    PacketStatus listen();
    void send();
    
    void set(TXIndex index, int8_t value);
    int8_t get(RXIndex index);
    int16_t getPosZ();
    
private:
    USARTClass &_rxSerialPort;
    USARTClass &_txSerialPort;
    RXPacket _rxPacket;
    TXPacket _txPacket;
    
};


#endif /* defined(____PacketController__) */
