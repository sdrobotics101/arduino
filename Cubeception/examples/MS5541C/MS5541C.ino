#include <Wire.h>
#include <SPI.h>
#include <Cubeception.h>

MS5541C ms5541c;

void setup() {
  Serial.begin(115200);
  SPI.begin();
  ms5541c.begin();

}

void loop() {
  ms5541c.queueD2();
  delay(35);
  ms5541c.readD2();
  ms5541c.queueD1();
  delay(35);
  ms5541c.readD1();
 
  Serial.println(ms5541c.getTemperature());
  Serial.println(ms5541c.getPressure());
  

}
