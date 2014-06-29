//
//  PacketController.h
//  
//
//  Created by Rahul Salvi on 6/28/14.
//
//

#ifndef ____PacketController__
#define ____PacketController__

#include <iostream>
#include "Arduino.h"
#include "RXPacket.h"
#include "TXPacket.h"

class PacketController {
public:
    
    bool listen();
    void send();
    
    
    
};

#endif /* defined(____PacketController__) */
