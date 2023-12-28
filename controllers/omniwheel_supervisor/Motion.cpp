#include "Motion.hpp"

PID::PID() {
    min_out = 0, max_out = 0, min_integral = 0, max_integral = 0;
    integral = 0, derivative = 0;
    prev_error = 0;
}

float PID::calculatePID(float error, float min_max) {
    min_out = min_integral = -min_max;
    max_out = max_integral = min_max;

    float proportional = kp * error;
    integral += ki * error * time_pid;
    derivative = kd * (error - prev_error) / time_pid;

    if (integral > max_integral)
        integral = max_integral;
    else if (integral < min_integral)
        integral = min_integral;

    output_speed = proportional + integral + derivative;
    if (output_speed > max_out)
        output_speed = max_out;
    else if (output_speed < min_out)
        output_speed = min_out;

    prev_error = error;
    return output_speed;
}

void PID::setParam(float kp_, float ki_, float kd_) {
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

void PID::reset() {
    integral = 0, derivative = 0;
    prev_error = 0;
    output_speed = 0;
}

Motion::Motion() {
    position_pid = new PID();
    yaw_pid = new PID();
}

void Motion::accelControl(Point2D *output_vel, Point2D &data) {
    // Static Velocity Buffer to store previous velocity.
    static float v_buffer[2];

    // Calculate Change in velocity
    float delta_v[2];
    delta_v[0] = data.x - v_buffer[0];
    delta_v[1] = data.y - v_buffer[1];

    // Convert the change to Polar Representation
    float r = sqrt(delta_v[0] * delta_v[0] + delta_v[1] * delta_v[1]);
    float theta = atan2(delta_v[1], delta_v[0]);
    // limit acceleration magnitude
    if (r > 3) r = 3;

    // Modify new velocity by converting back to Cartesian Representation
    v_buffer[0] += r * cos(theta);
    v_buffer[1] += r * sin(theta);

    // Set Output Parameters
    output_vel->x = v_buffer[0];
    output_vel->y = v_buffer[1];
    output_vel->theta = data.theta;

    // IC(output_vel->x, output_vel->y, output_vel->theta);
}

void Motion::basicMotion(float vx, float vy, float vtheta, float thetaRobot,
                         Point2D &output) {
    Point2D dataInput;
    float velout[3];
    velout[0] = cos(thetaRobot) * vx + sin(thetaRobot) * vy;
    velout[1] = -sin(thetaRobot) * vx + cos(thetaRobot) * vy;
    // IC(thetaRobot);
    dataInput.x = velout[0];
    dataInput.y = velout[1];
    dataInput.theta = vtheta;
    accelControl(&output, dataInput);
}

void Motion::positionAngularControl(double &errorX, double &errorY,
                                    double &errorTheta, double yaw,
                                    Point2D &outMotor) {
    position_pid->setParam(400, 200, 0);  // Set position_pid
    yaw_pid->setParam(0.5, 1, 0);         // Set yaw_pid

    error[0] = errorX;  // Displacement along global X axis remaining
    error[1] = errorY;  // Displacement global Y axis remaining
    error[2] = sqrt(error[0] * error[0] +
                    error[1] * error[1]);  // Net Displacement Magnitude
    error[3] = errorTheta;                 // Yaw Error

    // Apply PID on Error Magnitude
    output[2] = position_pid->calculatePID(error[2], 100);
    // Apply PID on Yaw Error
    output[3] = yaw_pid->calculatePID(error[3] * M_PI / 180.0, 5);

    // Break into components
    output[0] = output[2] * cos(atan2(error[1], error[0]));
    output[1] = output[2] * sin(atan2(error[1], error[0]));

    // IC(output[0], output[1], output[2], output[3]);
    basicMotion(output[0], output[1], output[3], yaw, outMotor);
}
