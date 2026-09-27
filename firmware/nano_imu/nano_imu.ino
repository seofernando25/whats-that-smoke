#include <Arduino_BMI270_BMM150.h>
#include <Wire.h>

bool readRegs(uint8_t reg, uint8_t *data, uint8_t count) {
  Wire1.beginTransmission(0x68);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom(0x68, (int)count) != count) return false;
  for (uint8_t index = 0; index < count; index++) data[index] = Wire1.read();
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!IMU.begin(BOSCH_ACCELEROMETER_ONLY)) {
    Serial.println("{\"type\":\"imu_error\",\"error\":\"bmi270_init_failed\"}");
    delay(1000);
  }
  Wire1.setClock(400000);
}

void loop() {
  uint8_t status, data[12];
  if (!readRegs(0x03, &status, 1) || !(status & 0x40) || !readRegs(0x0C, data, 12)) return;
  int16_t raw[6];
  memcpy(raw, data, sizeof(raw));
  const float ax = raw[0] / 8192.0f, ay = raw[1] / 8192.0f, az = raw[2] / 8192.0f;
  const float gx = raw[3] / 16.384f, gy = raw[4] / 16.384f, gz = raw[5] / 16.384f;

  Serial.print("{\"type\":\"imu\",\"t_ms\":");
  Serial.print(micros());
  Serial.print(",\"a\":[");
  Serial.print(ax, 5); Serial.print(','); Serial.print(ay, 5); Serial.print(','); Serial.print(az, 5);
  Serial.print("],\"g\":[");
  Serial.print(gx, 4); Serial.print(','); Serial.print(gy, 4); Serial.print(','); Serial.print(gz, 4);
  Serial.println("]}");
}
