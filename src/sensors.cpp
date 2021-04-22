#include <sensors.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#include <globals.h>
#include <flight_controller.h>

Adafruit_HMC5883_Unified mag;
SFE_UBLOX_GPS gps;

#ifdef TARGET_TEENSY35
MPU6050 mpu(0x68);
Adafruit_BMP280 bmp;
#endif

#ifdef TARGET_FCU1062
ICP101xx icp;      // Barometer
ICM_20948_I2C icm; // 9DOF Motion Sensor
ICM_20948_SPI icm_spi;
#endif

float relative_altitude = 0;
float airpress_abs = 0;
float gndlvl_airpressure = 0;
float temp_barometer = 0;
float temp_imu = 0;
float temperature_avg = 0;

float gps_longitude = 0;
float gps_latitude = 0;
float gps_altitude = 0;
float gps_heading = 0;
float gps_gnd_speed = 0;

void initAllSensors()
{
    /* GPS */
    /*
    gps_uart.begin(9600);
    if (gps.begin(gps_uart) == true)
    {
        Serial.println("GPS connected at baudrate 460800");
    }
    else
    {
        gps_uart.begin(9600);
        if (gps.begin(gps_uart) == true)
        {
            Serial.println("GPS: connected at 9600 baud, switching to 460800");
            gps.setSerialRate(9600);
            gps_uart.begin(9600);
            delay(100);
        }
    }

    gps.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
    gps.setI2COutput(COM_TYPE_UBX);   //Set the I2C port to output UBX only (turn off NMEA noise)
    gps.saveConfiguration();          //Save the current settings to flash and BBR
*/
#ifdef TARGET_TEENSY35
    /* Pressure Sensor  */
    bmp.begin(0x76);
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,  /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X16, /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16    /* Filtering. */
    );

    gndlvl_airpressure = bmp.readPressure();

    /*  Motion Sensor  */
    mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500); // 500 dps
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // 2 g
    mpu.setSleepEnabled(false);
    
    mpu.setXAccelOffset(-2015);
    mpu.setYAccelOffset(-1741);
    mpu.setZAccelOffset(1843);
    mpu.setXGyroOffset(9);
    mpu.setYGyroOffset(31);
    mpu.setZGyroOffset(33);
#endif

#ifdef TARGET_FCU1062
    /* Pressure Sensor  */
    icp.begin();
    //FAST: ~3ms, NORMAL: ~7ms (default), ACCURATE: ~24ms, VERY_ACCURATE: ~95ms
    icp.measureStart(icp.VERY_ACCURATE);
    while (!icp.dataReady())
    {
        static uint64_t loopstart = millis();
        if (millis() - loopstart > 200)
        {
            Serial.println("Pressure sensor not found");
            break;
        }
    }
    if (icp.dataReady())
    {
        gndlvl_airpressure = icp.getPressurePa() / 100;
    }

    /*  Motion Sensor  */
    icm.begin();
    icm.swReset();
    if (icm.status != ICM_20948_Stat_Ok)
    {
        Serial.print(F("Software Reset returned: "));
        Serial.println(icm.statusString());
    }
    delay(250);
    icm.sleep(false);
    icm.lowPower(false);
    // Sample configuration
    icm.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
    if (icm.status != ICM_20948_Stat_Ok)
    {
        Serial.print(F("setSampleMode returned: "));
        Serial.println(icm.statusString());
    }
    // Scale configuration
    ICM_20948_fss_t f_scale_settings;
    f_scale_settings.a = gpm2;    // 2G acceleration scale
    f_scale_settings.g = dps2000; // 2000 degrees/second gyro scale
    icm.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), f_scale_settings);
    if (icm.status != ICM_20948_Stat_Ok)
    {
        Serial.print(F("setFullScale returned: "));
        Serial.println(icm.statusString());
    }
    // Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t low_pass_filter;
    low_pass_filter.a = acc_d473bw_n499bw;
    low_pass_filter.g = gyr_d361bw4_n376bw5;
    icm.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), low_pass_filter);
    if (icm.status != ICM_20948_Stat_Ok)
    {
        Serial.print(F("setDLPcfg returned: "));
        Serial.println(icm.statusString());
    }

#endif
}

void updateGps()
{
    gps_latitude = gps.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(gps_latitude);

    gps_longitude = gps.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(gps_longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    gps_altitude = gps.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(gps_altitude);
    Serial.print(F(" (mm)"));

    gps_heading = gps.getHeading();
    gps_gnd_speed = gps.getGroundSpeed();

    gps_siv = gps.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(gps_siv);
}

void updateImu()
{
#ifdef TARGET_TEENSY35
    mpu.getMotion6(&acc_x, &acc_y, &acc_z, &gyr_x, &gyr_y, &gyr_z);
    temp_imu = mpu.getTemperature();
#endif

#ifdef TARGET_FCU1062

    /* IMU */
    if (icm.dataReady())
    {
        icm.getAGMT(); // update values
        acc_x = icm.accX();
        acc_y = icm.accY();
        acc_z = icm.accZ();

        gyr_x = icm.gyrX();
        gyr_y = icm.gyrY();
        gyr_z = icm.gyrZ();

        mag_x = icm.magX();
        mag_y = icm.magY();
        mag_z = icm.magZ();

        temp_imu = icm.temp();
    }

#endif
}

void updateBarometer()
{
#ifdef TARGET_FCU1062
    /* Barometer */
    icp.measureStart(icp.ACCURATE);
    if (icp.dataReady())
    {
        airpress_abs = icp.getPressurePa();
        temp_barometer = icp.getTemperatureC();
    }
#endif

#ifdef TARGET_TEENSY35
    /* Barometer */
    airpress_abs = bmp.readPressure();
    temp_barometer = bmp.readTemperature();

    relative_altitude = ((pow((gndlvl_airpressure / airpress_abs), (1 / 5.257)) - 1) * (temp_barometer + 273.15)) / 0.0065;
    temperature_avg = (temp_imu + temp_barometer) / 2;

#endif
}

void updateMagnetoMeter()
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
