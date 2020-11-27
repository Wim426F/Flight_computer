#ifndef FLIGHT_CONTROLS_H_
#define FLIGHT_CONTROLS_H_

class ControlSystem
{
private:
    int pitch_correction();
    int roll_correction();
    int yaw_correction();

public:
    bool disableWaypoint;

    void initialize();
    void calibrate();
    void steer();
    void hover(float altitude = 0);
    void enableWaypoint(float longitude, float latitude, float altitude);
};

#endif
