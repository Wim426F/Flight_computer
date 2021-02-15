#ifndef FLIGHT_CONTROLS_H_
#define FLIGHT_CONTROLS_H_


void initController();
void calibrateAngles();
void mainControl();
void enableWaypoint(float longitude, float latitude, float altitude);

extern bool hold_position;
extern float hold_altitude;


#endif
