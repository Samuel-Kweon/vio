#include "vio/ekf.hpp"

EKF::EKF(const Eigen::VectorXd& x_init, const Eigen::MatrixXd& P_init, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,MotionFn f, MotionJacobian F, MeasureFn h, MeasureJacobian H):
    x_(x_init), P_(P_init), Q_(Q), R_(R), f_(f), F_(F), h_(h), H_(H){
    };

void EKF::predict(const Eigen::VectorXd& u){
    Eigen::MatrixXd F = F_(x_, u);
    x_ = f_(x_, u);
    P_ = F * P_ * F.transpose() + Q_;
}

void EKF::update(const Eigen::VectorXd& z){
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.rows(), x_.rows());
    Eigen::MatrixXd H = H_(x_);
    Eigen::VectorXd y = z - h_(x_);
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = (I - K * H) * P_;
}