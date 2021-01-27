#ifndef FLIGHT_CONTROLS_H_
#define FLIGHT_CONTROLS_H_


void init_controller();
void calibrate();
void control_loop();

extern bool hold_position;
extern float hold_altitude;
void enableWaypoint(float longitude, float latitude, float altitude);

#endif
