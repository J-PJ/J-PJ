/* This example reads the raw values from the gyro on the Zumo 32U4, and prints those raw
values to the serial monitor.

The gyro readings can be converted to degrees per second (dps)
using the sensitivity specified in the L3GD20H or LSM6DS33
datasheet.  The default sensitivity is 8.75 mdps/LSB (least-
significant bit).  A raw reading of 10285 would correspond to
90 dps.*/

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4IMU imu;

char report[120];

void setup()
{
  Wire.begin();

  if (!imu.init())
  {
    // Failed to detect the compass.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to initialize IMU sensors."));
      delay(100);
    }
  }

  imu.enableDefault();
}
float gyroSensitivity = 8.7506;

void loop()
{
  imu.read();

  snprintf_P(report, sizeof(report),
    PSTR("G: %6d %6d %6d"),
    
    
    
    (imu.g.x * gyroSensitivity / 1000.0), (imu.g.y * gyroSensitivity / 1000.0), (imu.g.z * gyroSensitivity / 1000.0));
    Serial.println(report);

  delay(100);
}
