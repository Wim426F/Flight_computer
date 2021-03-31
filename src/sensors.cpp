#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#include <globals.h>

Adafruit_HMC5883_Unified mag;

MPU6050 mpu(0x68);
Adafruit_BMP280 bmp;

void initAllSensors()
{
}

void getGps()
{
}

void getImu()
{
#ifdef TARGET_TEENSY35

    mpu.getMotion6(&acc_x, &acc_y, &acc_z, &gyr_x, &gyr_y, &gyr_z);

#endif

#ifdef TARGET_FCU1062

#endif
}

void getMagnetoMeter()
{
    sensors_event_t event;
    mag.getEvent(&event);

    Serial.print(event.magnetic.x);
    Serial.print(event.magnetic.y);
    Serial.print(event.magnetic.z);

    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(event.magnetic.y, event.magnetic.x);

    float declinationAngle = 2;
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if (heading < 0)
        heading += 2 * PI;

    // Check for wrap due to addition of declination.
    if (heading > 2 * PI)
        heading -= 2 * PI;

    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180 / M_PI;

    Serial.print("Heading (degrees): ");
    Serial.println(headingDegrees);
}
