#ifndef EKFpilot_H
#define EKFpilot_H

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter();
    void begin();
    void setGPSData(float latitude, float longitude);
    void setIMUData(float ax, float ay);
    float getEstimatedX() const;
    float getEstimatedY() const;

private:
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd F;
    Eigen::MatrixXd H;

    void predict();
    void update(const Eigen::VectorXd &z);
};

KalmanFilter::KalmanFilter() {
    x = Eigen::VectorXd(4);
    P = Eigen::MatrixXd(4, 4);
    Q = Eigen::MatrixXd(4, 4);
    R = Eigen::MatrixXd(2, 2);
    F = Eigen::MatrixXd(4, 4);
    H = Eigen::MatrixXd(2, 4);
}

void KalmanFilter::begin() {
    x << 0, 0, 0, 0;
    P.setIdentity();
    P *= 1000;
    R = Eigen::MatrixXd::Identity(2, 2) * 5;
    Q = Eigen::MatrixXd::Identity(4, 4) * 0.1;
}

void KalmanFilter::setGPSData(float latitude, float longitude) {
    Eigen::VectorXd z(2);
    z << latitude, longitude;
    update(z);
}

void KalmanFilter::setIMUData(float ax, float ay) {
    float dt = 0.1;
    F << 1, dt, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, dt,
         0, 0, 0, 1;

    x(0) += x(1) * dt + 0.5 * ax * dt * dt;
    x(2) += x(3) * dt + 0.5 * ay * dt * dt;
    x(1) += ax * dt;
    x(3) += ay * dt;
}

void KalmanFilter::predict() {
    x = F * x;
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd &z) {
    Eigen::VectorXd y = z - H * x;
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    x += K * y;
    P = (Eigen::MatrixXd::Identity(4, 4) - K * H) * P;
}

float KalmanFilter::getEstimatedX() const {
    return x(0);
}

float KalmanFilter::getEstimatedY() const {
    return x(2);
}

#endif
