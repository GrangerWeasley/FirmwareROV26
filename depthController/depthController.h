#ifndef DEPTH_H
#define DEPTH_H
#include "B:\AUR\ROV\PID\pid\pid.h"

enum class direction
{
    STOP,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    UP,
    DOWN,
    ROTATE_CW,
    ROTATE_CCW
};

class depthController
{
private:
    PID depthPID;
    direction dir = direction::STOP;

public:
    depthController(double kp, double ki, double kd);
    double update(double setpoint, double depth, double dt);
    enum class direction get_direction();
};

#endif