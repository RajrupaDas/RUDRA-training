#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "EKFpilot.h"

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    void update(float setpoint, float current_position, float dt);
    void setSensorData(float latitude, float longitude, float ax, float ay);
    float getOutput() const;

private:
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    EKFpilot kalman;
    float output;
};

PIDController::PIDController(float kp, float ki, float kd) 
    : kp(kp), ki(ki), kd(kd), integral(0), last_error(0), output(0) {
    kalman.begin();
}

void PIDController::update(float setpoint, float current_position, float dt) {
    float estimated_position = kalman.getEstimatedX();
    float error = setpoint - estimated_position;
    integral += error * dt;
    float derivative = (error - last_error) / dt;
    last_error = error;
    output = (kp * error) + (ki * integral) + (kd * derivative);
}

void PIDController::setSensorData(float latitude, float longitude, float ax, float ay) {
    kalman.setGPSData(latitude, longitude);
    kalman.setIMUData(ax, ay);
}

float PIDController::getOutput() const {
    return output;
}

#endif // PIDCONTROLLER_H
