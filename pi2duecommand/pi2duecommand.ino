#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <TXPacket.h>
#include <RXPacket.h>
#include <PacketController.h>

Adafruit_PWMServoDriver pwmU1(0x79);
Adafruit_PWMServoDriver pwmU2(0x71);

PacketController packetController;
PacketStatus pStatus = NO_HEADER_FOUND; 

int MXR1 = 15;
int MXR2 = 14;
int MXR3 = 13;
int MXR4 = 12;

int MXF1 = 11;
int MXF2 = 10;
int MXF3 = 9;
int MXF4 = 8;

int MYR1 = 7;
int MYR2 = 6;
int MYR3 = 5;
int MYR4 = 4;

int MYF1 = 3;
int MYF2 = 2;
int MYF3 = 1;
int MYF4 = 0;

int MZR1 = 15;
int MZR2 = 14;
int MZR3 = 13;
int MZR4 = 12;

int MZF1 = 11;
int MZF2 = 10;
int MZF3 = 9;
int MZF4 = 8;

int16_t velX;
int16_t velY;
int16_t velZ;

int i = 0;
int pin = 22;

void setup() {
  
  packetController.begin();
  
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  
  pwmU1.begin();
  pwmU2.begin();
  
  pwmU1.setPWMFreq(1600);
  pwmU2.setPWMFreq(1600);
  
  pwmU1.setPWM(MXF1, 0, 0);
  pwmU1.setPWM(MXF2, 0, 0);
  pwmU1.setPWM(MXF3, 0, 0);
  pwmU1.setPWM(MXF4, 0, 0);
  
  pwmU1.setPWM(MXR1, 0, 0);
  pwmU1.setPWM(MXR2, 0, 0);
  pwmU1.setPWM(MXR3, 0, 0);
  pwmU1.setPWM(MXR4, 0, 0);
  
  pwmU1.setPWM(MYF1, 0, 0);
  pwmU1.setPWM(MYF2, 0, 0);
  pwmU1.setPWM(MYF3, 0, 0);
  pwmU1.setPWM(MYF4, 0, 0);
    
  pwmU1.setPWM(MYR1, 0, 0);
  pwmU1.setPWM(MYR2, 0, 0);
  pwmU1.setPWM(MYR3, 0, 0);
  pwmU1.setPWM(MYR4, 0, 0);
  
  pwmU2.setPWM(MZF1, 0, 0);
  pwmU2.setPWM(MZF2, 0, 0);
  pwmU2.setPWM(MZF3, 0, 0);
  pwmU2.setPWM(MZF4, 0, 0);
    
  pwmU2.setPWM(MZR1, 0, 0);
  pwmU2.setPWM(MZR2, 0, 0);
  pwmU2.setPWM(MZR3, 0, 0);
  pwmU2.setPWM(MZR4, 0, 0);
  
}

void loop() {
  //digitalWrite(pin, HIGH);
  while (packetController.listen() != VALID_PACKET) {
    ;
  }
  /*  
  if (i > 100) {
    //pwmU1.reset();
    //pwmU2.reset();
    i = 0;
  }  
    
  i++;
  */ 
   
  //digitalWrite(pin, LOW); 
   
  velX = (packetController.get8(VELX)) * 32;
  velY = (packetController.get8(VELY)) * 32;
  velZ = (packetController.get8(VELZ)) * 32;
  
  
  /*
  Serial.println("VALID PACKET ");
  
  Serial.print(pwmU1.getMode1(), HEX); Serial.print(" ");
  Serial.println(pwmU1.getMode2(), HEX);
  Serial.print(pwmU2.getMode1(), HEX); Serial.print(" ");
  Serial.println(pwmU2.getMode2(), HEX);
  
  Serial.print(velX / 32); Serial.print(" ");
  Serial.print(velY / 32); Serial.print(" ");
  Serial.print(velZ / 32); Serial.print(" ");
  Serial.println("");
  */
  
  
  digitalWrite(pin, HIGH);
  
  //pwmU1.setMode1(0x21);
  //pwmU1.setMode2(0x04);
  //pwmU2.setMode1(0x21);
  //pwmU2.setMode2(0x04);
  
  if (velX > 0) {
    pwmU1.setPWM(MXR1, 0, 0);
    pwmU1.setPWM(MXR2, 0, 0);
    pwmU1.setPWM(MXR3, 0, 0);
    pwmU1.setPWM(MXR4, 0, 0);
    
    pwmU1.setPWM(MXF1, MXF1*256, ((MXF1*256) + velX) % 4096);
    pwmU1.setPWM(MXF2, MXF2*256, ((MXF2*256) + velX) % 4096);
    pwmU1.setPWM(MXF3, MXF3*256, ((MXF3*256) + velX) % 4096);
    pwmU1.setPWM(MXF4, MXF4*256, ((MXF4*256) + velX) % 4096);
  } else if (velX < 0) {
    
    velX *= -1;
    
    pwmU1.setPWM(MXF1, 0, 0);
    pwmU1.setPWM(MXF2, 0, 0);
    pwmU1.setPWM(MXF3, 0, 0);
    pwmU1.setPWM(MXF4, 0, 0);
    
    pwmU1.setPWM(MXR1, MXR1*256, ((MXR1*256) + velX) % 4096);
    pwmU1.setPWM(MXR2, MXR2*256, ((MXR2*256) + velX) % 4096);
    pwmU1.setPWM(MXR3, MXR3*256, ((MXR3*256) + velX) % 4096);
    pwmU1.setPWM(MXR4, MXR4*256, ((MXR4*256) + velX) % 4096);
  } else {
    pwmU1.setPWM(MXF1, 0, 0);
    pwmU1.setPWM(MXF2, 0, 0);
    pwmU1.setPWM(MXF3, 0, 0);
    pwmU1.setPWM(MXF4, 0, 0);
    
    pwmU1.setPWM(MXR1, 0, 0);
    pwmU1.setPWM(MXR2, 0, 0);
    pwmU1.setPWM(MXR3, 0, 0);
    pwmU1.setPWM(MXR4, 0, 0);
  }
  
  if (velY > 0) {
    pwmU1.setPWM(MYR1, 0, 0);
    pwmU1.setPWM(MYR2, 0, 0);
    pwmU1.setPWM(MYR3, 0, 0);
    pwmU1.setPWM(MYR4, 0, 0);
    
    pwmU1.setPWM(MYF1, MYF1*256, ((MYF1*256) + velY) % 4096);
    pwmU1.setPWM(MYF2, MYF2*256, ((MYF2*256) + velY) % 4096);
    pwmU1.setPWM(MYF3, MYF3*256, ((MYF3*256) + velY) % 4096);
    pwmU1.setPWM(MYF4, MYF4*256, ((MYF4*256) + velY) % 4096);
  } else if (velY < 0) {
    
    velY *= -1;
    
    pwmU1.setPWM(MYF1, 0, 0);
    pwmU1.setPWM(MYF2, 0, 0);
    pwmU1.setPWM(MYF3, 0, 0);
    pwmU1.setPWM(MYF4, 0, 0);
    
    pwmU1.setPWM(MYR1, MYR1*256, ((MYR1*256) + velY) % 4096);
    pwmU1.setPWM(MYR2, MYR2*256, ((MYR2*256) + velY) % 4096);
    pwmU1.setPWM(MYR3, MYR3*256, ((MYR3*256) + velY) % 4096);
    pwmU1.setPWM(MYR4, MYF4*256, ((MYR4*256) + velY) % 4096);
  } else {
    pwmU1.setPWM(MYF1, 0, 0);
    pwmU1.setPWM(MYF2, 0, 0);
    pwmU1.setPWM(MYF3, 0, 0);
    pwmU1.setPWM(MYF4, 0, 0);
    
    pwmU1.setPWM(MYR1, 0, 0);
    pwmU1.setPWM(MYR2, 0, 0);
    pwmU1.setPWM(MYR3, 0, 0);
    pwmU1.setPWM(MYR4, 0, 0);
  }
  
  if (velZ > 0) {
    pwmU2.setPWM(MZR1, 0, 0);
    pwmU2.setPWM(MZR2, 0, 0);
    pwmU2.setPWM(MZR3, 0, 0);
    pwmU2.setPWM(MZR4, 0, 0);
    
    pwmU2.setPWM(MZF1, MZF1*256, ((MZF1*256) + velZ) % 4096);
    pwmU2.setPWM(MZF2, MZF2*256, ((MZF2*256) + velZ) % 4096);
    pwmU2.setPWM(MZF3, MZF3*256, ((MZF3*256) + velZ) % 4096);
    pwmU2.setPWM(MZF4, MZF4*256, ((MZF4*256) + velZ) % 4096);
  } else if (velZ < 0) {
    
    velZ *= -1;
    
    pwmU2.setPWM(MZF1, 0, 0);
    pwmU2.setPWM(MZF2, 0, 0);
    pwmU2.setPWM(MZF3, 0, 0);
    pwmU2.setPWM(MZF4, 0, 0);
    
    pwmU2.setPWM(MZR1, MZR1*256, ((MZR1*256) + velZ) % 4096);
    pwmU2.setPWM(MZR2, MZR2*256, ((MZR2*256) + velZ) % 4096);
    pwmU2.setPWM(MZR3, MZR3*256, ((MZR3*256) + velZ) % 4096);
    pwmU2.setPWM(MZR4, MZR4*256, ((MZR4*256) + velZ) % 4096);
  } else {
    pwmU2.setPWM(MZF1, 0, 0);
    pwmU2.setPWM(MZF2, 0, 0);
    pwmU2.setPWM(MZF3, 0, 0);
    pwmU2.setPWM(MZF4, 0, 0);
    
    pwmU2.setPWM(MZR1, 0, 0);
    pwmU2.setPWM(MZR2, 0, 0);
    pwmU2.setPWM(MZR3, 0, 0);
    pwmU2.setPWM(MZR4, 0, 0);
  }
  
  digitalWrite(pin, LOW);
  
}
