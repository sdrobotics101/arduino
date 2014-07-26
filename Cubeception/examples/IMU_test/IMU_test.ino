#include <Wire.h>
#include <SPI.h>
#include <Cubeception.h>

MPU6050 mpu9150(MPU_ADDR);

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  mpu9150.initialize();
}

void loop() {
  mpu9150.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  
  Serial.print(ax); Serial.print(" ");
  Serial.print(ay); Serial.print(" ");
  Serial.print(az); Serial.print(" ");
  Serial.print(gx); Serial.print(" ");
  Serial.print(gy); Serial.print(" ");
  Serial.print(gz); Serial.print(" ");
  Serial.print(mx); Serial.print(" ");
  Serial.print(my); Serial.print(" ");
  Serial.print(mz); Serial.print(" ");
  Serial.println("");
  
}
