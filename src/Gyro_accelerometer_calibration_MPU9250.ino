#include <Wire.h>

// MPU9250 Registers and Configuration
#define MPU9250_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define INT_PIN_CFG 0x37
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

int16_t AccXLSB, AccYLSB, AccZLSB;
int16_t GyroX, GyroY, GyroZ;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
volatile float RatePitch, RateRoll, RateYaw;
volatile float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
volatile float AccZCalibration, AccYCalibration, AccXCalibration;
int RateCalibrationNumber;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  
  delay(100);
  
  // Initialize the MPU9250 sensor
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00); // Wake up MPU9250
  Wire.endTransmission(true);
  
  delay(100);
  
  // Configure accelerometer and gyroscope
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0x00); // ±8g range
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x08); // ±500 dps range
  Wire.endTransmission();
  
  // Enable I2C bypass mode for magnetometer (optional)
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(INT_PIN_CFG);
  Wire.write(0x02); // Bypass enabled
  Wire.endTransmission(true);
  
  delay(100);
  
  // Calibration instructions
  Serial.println("\n========================================");
  Serial.println("Welcome to MPU9250 IMU Calibration");
  Serial.println("========================================");
  Serial.println("Place the quadcopter flat on a level surface");
  Serial.println("Press 'Space bar' and then 'Enter' to begin calibration...");
  while (!Serial.available()) {}
  Serial.read();
  
  Serial.println("Calibrating... Please keep the quadcopter still for 2 seconds");
  
  calibrateGyroSimple();
  
  Serial.println("\n========== CALIBRATION COMPLETE ==========");
  Serial.print("GyroX (Roll):  ");
  Serial.print(RateRoll);
  Serial.println(" deg/s");
  Serial.print("GyroY (Pitch): ");
  Serial.print(RatePitch);
  Serial.println(" deg/s");
  Serial.print("GyroZ (Yaw):   ");
  Serial.print(RateYaw);
  Serial.println(" deg/s");
  Serial.print("AccX:          ");
  Serial.print(AccX);
  Serial.println(" g");
  Serial.print("AccY:          ");
  Serial.print(AccY);
  Serial.println(" g");
  Serial.print("AccZ:          ");
  Serial.print(AccZ);
  Serial.println(" g");
  
  Serial.println("\n========== CALIBRATION VALUES ==========");
  Serial.print("RateCalibrationRoll=");
  Serial.print(RateCalibrationRoll, 4);
  Serial.println(";");
  Serial.print("RateCalibrationPitch=");
  Serial.print(RateCalibrationPitch, 4);
  Serial.println(";");
  Serial.print("RateCalibrationYaw=");
  Serial.print(RateCalibrationYaw, 4);
  Serial.println(";");
  Serial.print("AccXCalibration=");
  Serial.print(AccXCalibration, 4);
  Serial.println(";");
  Serial.print("AccYCalibration=");
  Serial.print(AccYCalibration, 4);
  Serial.println(";");
  Serial.print("AccZCalibration=");
  Serial.print(AccZCalibration, 4);
  Serial.println(";");
  Serial.println("========================================\n");
  
  Serial.println("Press 'Space bar' and then 'Enter' to print sensor values continuously");
  while (!Serial.available()) {}
  Serial.read();
}

void loop() {
  gyro_signals();
  
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  
  AccX -= AccXCalibration;
  AccY -= AccYCalibration;
  AccZ -= AccZCalibration;
  
  Serial.print("AccX: ");
  Serial.print(AccX, 3);
  Serial.print(" g, AccY: ");
  Serial.print(AccY, 3);
  Serial.print(" g, AccZ: ");
  Serial.print(AccZ, 3);
  Serial.print(" g | GyroX: ");
  Serial.print(RateRoll, 2);
  Serial.print(" dps, GyroY: ");
  Serial.print(RatePitch, 2);
  Serial.print(" dps, GyroZ: ");
  Serial.print(RateYaw, 2);
  Serial.println(" dps");
  
  delay(100);
}

void gyro_signals(void) {
  // Configure accelerometer (low-pass filter)
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(CONFIG);
  Wire.write(0x05); // 6Hz bandwidth
  Wire.endTransmission();
  
  // Read accelerometer data
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(MPU9250_ADDR, 6);
  
  AccXLSB = Wire.read() << 8 | Wire.read();
  AccYLSB = Wire.read() << 8 | Wire.read();
  AccZLSB = Wire.read() << 8 | Wire.read();
  
  // Configure gyroscope (low-pass filter)
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(CONFIG);
  Wire.write(0x05); // 6Hz bandwidth
  Wire.endTransmission();
  
  // Read gyroscope data
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(MPU9250_ADDR, 6);
  
  GyroX = Wire.read() << 8 | Wire.read();
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();
  
  // Convert raw values to physical units
  // MPU9250 ±8g range: 4096 LSB/g
  AccX = (float)AccXLSB / 4096.0;
  AccY = (float)AccYLSB / 4096.0;
  AccZ = (float)AccZLSB / 4096.0;
  
  // MPU9250 ±500 dps range: 65.5 LSB/dps
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  
  // Calculate Roll and Pitch angles
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29578;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29578;
}

void calibrateGyroSimple(void) {
  RateCalibrationRoll = 0;
  RateCalibrationPitch = 0;
  RateCalibrationYaw = 0;
  AccXCalibration = 0;
  AccYCalibration = 0;
  AccZCalibration = 0;
  
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    AccXCalibration += AccX;
    AccYCalibration += AccY;
    AccZCalibration += AccZ;
    
    delay(1);
  }
  
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  AccXCalibration /= 2000;
  AccYCalibration /= 2000;
  AccZCalibration /= 2000;
  AccZCalibration -= 1.0; // Remove gravity offset
}
