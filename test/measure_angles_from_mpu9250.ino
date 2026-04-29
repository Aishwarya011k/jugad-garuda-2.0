#include <Wire.h>

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;

#define MPU9250_ADDR 0x68

void gyro_signals(void) {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  
  Wire.requestFrom(MPU9250_ADDR, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x43);
  Wire.endTransmission();
  
  Wire.requestFrom(MPU9250_ADDR, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
  
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29578;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29578;
}

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  Wire.setClock(400000);
  Wire.begin();
  
  delay(250);
  
  Wire.beginTransmission(MPU9250_ADDR); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  // Enable I2C bypass mode
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();
  
  delay(100);
}

void loop() {
  gyro_signals();
  
  Serial.print("Acceleration X [g]= ");
  Serial.print(AccX, 3);
  Serial.print(" | Acceleration Y [g]= ");
  Serial.print(AccY, 3);
  Serial.print(" | Acceleration Z [g]= ");
  Serial.println(AccZ, 3);
  
  Serial.print("Roll Angle [deg]= ");
  Serial.print(AngleRoll, 2);
  Serial.print(" | Pitch Angle [deg]= ");
  Serial.println(AnglePitch, 2);
  
  Serial.print("GyroX [dps]= ");
  Serial.print(RateRoll, 2);
  Serial.print(" | GyroY [dps]= ");
  Serial.print(RatePitch, 2);
  Serial.print(" | GyroZ [dps]= ");
  Serial.println(RateYaw, 2);
  
  Serial.println("---");
  delay(100);
}
