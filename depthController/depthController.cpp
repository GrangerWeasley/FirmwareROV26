#include "depthController.h"

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
    depthController(double kp, double ki, double kd)
        : depthPID(kp, ki, kd, 100, -100)
    {
        depthPID.set_derivative_filter(0.1); // tau needs to be tuned for low pass filter
    }

    double update(double setpoint, double depth, double dt)
    {
        double pidOutput = depthPID.update(setpoint, depth, dt);
        if (pidOutput >= 0)
        {
            dir = direction::DOWN;
            return pidOutput;
        }

        else if (pidOutput < 0)
        {
            dir = direction::UP;
            return -pidOutput;
        }
    }

    enum class direction get_direction()
    {
        return dir;
    }
};
