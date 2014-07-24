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

#define TX_PACKET_SIZE 15

enum TXIndexU8 {
	BATV = 12
};

enum TXIndexS8 {
};

enum TXIndexU16 {
	POSZ = 8,
	HEALTH = 10,
	TXCHECKSUM = 13
};

enum TXIndexS16 {
	MAGX = 2,
	MAGY = 4,
	MAGZ = 6
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
