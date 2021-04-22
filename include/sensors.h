#ifndef SENSORS_H
#define SENSORS_H

extern float airpress_groundlvl;
extern float airpress_abs;
extern float relative_altitude;

extern float temp_barometer;
extern float temp_imu;
extern float temperature_avg;

extern float gps_longitude;
extern float gps_latitude;
extern float gps_altitude;
extern float gps_heading;
extern float gps_gnd_speed;
extern float gps_siv;


void initAllSensors();
void updateImu();
void updateMagnetoMeter();
void updateGps();
void updateBarometer();

#endif