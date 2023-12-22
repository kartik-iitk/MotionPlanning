#ifndef ROBOT_H
#define ROBOT_H

#include "Coordinate.hpp"
#include "Icecream.hpp"
#include "math.h"
#include "vector"

namespace robot {

class RobotKinematic {
   private:
    Point2D pos = {0, 0, 0};
    Point2D vel = {0, 0, 0};

    static RobotKinematic *instance;
    float a = 45;  // Angle between wheels
    float L =
        41.6;  // radius from center to wheel (cm); BaseLength 4,795.94 MM;
    float r_wheel = 6;  // wheel radius (cm)
    float circumference = 2 * M_PI * r_wheel;
    std::vector<double> Venc = {0, 0, 0, 0};
    std::vector<double> prevEnc = {0, 0, 0, 0};

   public:
    static RobotKinematic *getInstance();
    std::vector<double> encData = {0, 0, 0, 0};

    RobotKinematic(){};
    Point2D getPos();
    void forwardKinematics(Point2D &outForward, float s1, float s2, float s3,
                           float s4);
    void calculateOdometry(const double yaw);
    void inverseKinematics(wheelAngularVel &outputInverse, float vx, float vy,
                           float theta);
    double angleNormalize(double angle);

    void setInitialPosition(float x, float y, float theta);
};
};  // namespace robot

#endif