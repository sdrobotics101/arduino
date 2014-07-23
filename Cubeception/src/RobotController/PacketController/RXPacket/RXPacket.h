//
//  RXPacket.h
//  
//
//  Created by Rahul Salvi on 6/28/14.
//
//

#ifndef ____RXPacket__
#define ____RXPacket__

#include "Arduino.h"

#define RX_PACKET_SIZE 11

/**
 *  Status of the packet received
 */
enum PacketStatus {
    VALID_PACKET = 0,
    INVALID_PACKET = 1,
    NO_HEADER_FOUND = 2,
    NOT_ENOUGH_DATA = 3
};

enum RXIndexS8 {
	VELX = 0,
	VELY = 1,
	VELZ = 2,
	ROTZ = 3
};

enum RXIndexU8 {
	TORPEDOCTL = 4,
	SERVOCTL = 5,
	LEDCTL = 6
};

enum RXIndexS16 {
};

enum RXIndexU16 {
	MODE = 7,
	RXCHECKSUM = 9
};

/**
 *  Handles input from serial port
 */
class RXPacket {
    friend class PacketController;
private:
    RXPacket();
    void begin();
    
    PacketStatus readPacket(USARTClass serialPort);
    uint16_t computeChecksum();
    
    uint8_t _data[RX_PACKET_SIZE];
};

#endif /* defined(____RXPacket__) */
