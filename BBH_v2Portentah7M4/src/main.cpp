#ifdef CORE_CM4  // M4 core code handles IMU sampling and RPC transmission

#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>
#include "RPC.h"
#include "rtos.h"
#include "SerialRPC.h"
// #define Serial SerialRPC 

extern TwoWire Wire1;
Adafruit_LSM6DS3TRC imu;
using namespace std::chrono_literals;
void setup() {
  // RPC.begin();                // Initialize RPC on M4 ([forum.arduino.cc](https://forum.arduino.cc/t/portenta-rpc-internal-h-how-can-i-install-this-library/971395?utm_source=chatgpt.com))
  Wire1.begin();               // SDA1/SCL1 bus
  // Explicit address 0x6A for LSM6DS3TRC
  if (!imu.begin_I2C(0x6A, &Wire1)) while(1);
  // imu.setAccelRange(LSM6DS3TRC_ACCEL_RANGE_4_G);
  // imu.setGyroRange(LSM6DS3TRC_GYRO_RANGE_500_DPS);
  // RPC.println("M4 and IMU initialized on Wire1 (SDA1/SCL1)");

  Serial.begin(115200);
  while (!Serial) {}
  if (!SerialRPC.begin()) {
    RPC.println("Failed to initialize SerialRPC!");
    // handle error…
  }

  

}

void loop() {
  
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);
  int32_t ax_i = (int32_t)(accel.acceleration.x * 1000.0f);
  int32_t ay_i = (int32_t)(accel.acceleration.y * 1000.0f);
  int32_t az_i = (int32_t)(accel.acceleration.z * 1000.0f);
  int32_t gx_i = (int32_t)(gyro.gyro.x * 1000.0f);
  int32_t gy_i = (int32_t)(gyro.gyro.y * 1000.0f);
  int32_t gz_i = (int32_t)(gyro.gyro.z * 1000.0f);

  // Build one line with leading '|' delimiter:
  char buf[80];
  snprintf(buf, sizeof(buf),
           "|%ld,%ld,%ld,%ld,%ld,%ld",
           ax_i, ay_i, az_i, gx_i, gy_i, gz_i);

  // Send in one RPC transaction:
  SerialRPC.println(buf);

  rtos::ThisThread::sleep_for(10ms);
}
#endif // CORE_CM4