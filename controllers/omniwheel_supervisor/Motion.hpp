#ifndef MOTION_H
#define MOTION_H

#include "Coordinate.hpp"
#include "Icecream.hpp"
#include "vector"

// PID Controller
class PID {
   private:
    float min_out, max_out, min_integral, max_integral;
    float output_speed;
    float proportional, integral, derivative, prev_error;
    float kp, ki, kd;
    const float time_pid = 0.01;

   public:
    PID();
    float calculatePID(float error, float min_max);
    void setParam(float kp_, float ki_, float kd_);
    void reset();
};

// Control the Position and Yaw of the robot using PID Controllers.
class Motion {
   private:
    std::vector<double> output = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> error = {0, 0, 0, 0, 0};
    void basicMotion(float vx, float vy, float vtheta, float thetaRobot,
                     Point2D &output);
    void accelControl(Point2D *out, Point2D &data);

   public:
    PID *position_pid, *yaw_pid, *velocity_pid;
    Motion();
    void positionAngularControl(double &errorX, double &errorY,
                                double &errorTheta, double yaw,
                                Point2D &outMotor, int count1, int path_size);
};

#endif
