#include "vio/kalman.hpp"



KalmanFilter::KalmanFilter(const Eigen::VectorXd& x_init,
                           const Eigen::MatrixXd& P_init,
                           const Eigen::MatrixXd& F,
                           const Eigen::MatrixXd& H,
                           const Eigen::MatrixXd& Q,
                           const Eigen::MatrixXd& R)
    : x_(x_init), P_(P_init), F_(F), H_(H), Q_(Q), R_(R)
{};

void KalmanFilter::predict(){
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd& z){
    Eigen::VectorXd y = z - H_ * x_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + K * y;
    Eigen::MatrixXd I = Eigen::Matrix4d::Identity();
    P_ = (I - K * H_) * P_;
}
