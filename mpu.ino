#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
unsigned long last_time = 0;
float roll = 0, pitch = 0;

void setup() {
  Serial.begin(250000);
  Wire.begin();
  
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while(1);
  }
  
  Serial.println("MPU6050 initialized");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Normalizacja i obliczenia
  float accel_x = ax / 16384.0;
  float accel_y = ay / 16384.0;
  float accel_z = az / 16384.0;
  
  float acc_pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * 180/PI;
  float acc_roll = atan2(accel_y, accel_z) * 180/PI;
  
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;
  last_time = current_time;
  
  float gyro_x = gx / 131.0;
  float gyro_y = gy / 131.0;
  
  float alpha = 0.96;
  pitch = alpha * (pitch + gyro_y * dt) + (1 - alpha) * acc_pitch;
  roll = alpha * (roll + gyro_x * dt) + (1 - alpha) * acc_roll;
  
  // Wysyłamy tylko wartości liczbowe oddzielone przecinkami
  Serial.print(roll);
  Serial.print(",");
  Serial.println(pitch);
  
  delay(10);
}
