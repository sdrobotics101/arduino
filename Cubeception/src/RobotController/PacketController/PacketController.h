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
#include "RXPacket/RXPacket.h"
#include "TXPacket/TXPacket.h"

/**
 *  Handles all packet I/O
 */
class PacketController {
public:
    PacketController(USARTClass &rxSerialPort = Serial3,
                     USARTClass &txSerialPort = Serial3);
    void begin(int baudRate = 115200);
    
    PacketStatus listen();
    void send();
    
    void setS8(TXIndexS8 index, int8_t value);
	void setU8(TXIndexU8 index, uint8_t value);
	void setS16(TXIndexS16 index, int16_t value);
	void setU16(TXIndexU16 index, uint16_t value);
	
	
    int8_t getS8(RXIndexS8 index);
	uint8_t getU8(RXIndexU8 index);
    int16_t getS16(RXIndexS16 index);
	uint16_t getU16(RXIndexU16 index);
    
private:
    USARTClass &_rxSerialPort;
    USARTClass &_txSerialPort;
    RXPacket _rxPacket;
    TXPacket _txPacket;
    
};


#endif /* defined(____PacketController__) */
