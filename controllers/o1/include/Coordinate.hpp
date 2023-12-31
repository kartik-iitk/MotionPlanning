#ifndef COORDINATE_H
#define COORDINATE_H

// To store the angular velocities of the 4 wheels.
struct wheelAngularVel {
    float w1, w2, w3, w4;
};

// To store (x position parameter, y position parameter, yaw parameter) which
// can either be the coordinate/velocity/acceleration depending on need.
struct Point2D {
    float x, y, theta;
    Point2D() {}
    Point2D(float x, float y, float z) : x(x), y(y), theta(z) {}
};

#endif