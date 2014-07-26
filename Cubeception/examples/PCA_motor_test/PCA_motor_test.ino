#include <Wire.h>
#include <SPI.h>
#include <Cubeception.h>

Adafruit_PWMServoDriver pwmU1(PWMU1_ADDR);
Adafruit_PWMServoDriver pwmU2(PWMU2_ADDR);

int pin = 22;

void setup() {
  Serial.begin(115200);
  
  pwmU1.begin();
  pwmU2.begin();
  
  pwmU1.setPWMFreq(1600);
  pwmU2.setPWMFreq(1600);
  
  for (int i = 15; i > -1; i--) {
    pwmU1.setPWM(i, 0, 0);
  }
  for (int i = 15; i > 7; i--) {
    pwmU2.setPWM(i, 0, 0);
  }
  
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void loop() {
  
  for (int i = 15; i > -1; i--) {
    
    pwmU1.setPWM(i, 0, 2048);
    
    pulse();
    
    Serial.print("U1: "); Serial.println(i);
    Serial.println(pwmU1.getMode1(), HEX);
    Serial.println(pwmU1.getMode2(), HEX);
    
    delay(5000);
    pwmU1.setPWM(i, 0, 0);
    delay(1000);
  }
  for (int i = 15; i > 7; i--) {
    
    pwmU2.setPWM(i, 0, 2048);
    
    pulse();
    
    Serial.print("U2: "); Serial.println(i);
    Serial.println(pwmU2.getMode1(), HEX);
    Serial.println(pwmU2.getMode2(), HEX);
    delay(5000);
    pwmU2.setPWM(i, 0, 0);
    delay(1000);
  }
  
   
}

void pulse() {
  digitalWrite(pin, HIGH);
  delay(100);
  digitalWrite(pin, LOW); 
}
