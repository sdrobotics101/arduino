//
//  TXPacket.h
//  
//
//  Created by Rahul Salvi on 6/28/14.
//
//

#ifndef ____TXPacket__
#define ____TXPacket__

#include "Arduino.h"
#include <math.h>

#define TX_PACKET_SIZE 13

enum TXIndexU8 {
	BATV = 10
};

enum TXIndexS8 {
};

enum TXIndexU16 {
	POSZ = 6,
	HEALTH = 8,
	TXCHECKSUM = 11
};

enum TXIndexS16 {
	MAGX = 0,
	MAGY = 2,
	MAGZ = 4
};

/**
 *  Handles output to serial port
 */
class TXPacket {
    friend class PacketController;
private:
    TXPacket();
    void begin();
    
    void sendPacket(USARTClass serialPort);
    uint16_t computeChecksum();
    
    uint8_t _data[TX_PACKET_SIZE];
};


#endif /* defined(____TXPacket__) */
