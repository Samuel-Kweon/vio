#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cmath>

class KalmanFilter{
public:

    using MotionFn        = std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>;
    using MotionJacobian  = std::function<Eigen::MatrixXd(const Eigen::MatrixXd&, const Eigen::VectorXd&)>;
    using MeasureFn       = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
    using MeasureJacobian = std::function<Eigen::MatrixXd(const Eigen::MatrixXd&)>;   

    KalmanFilter(const Eigen::VectorXd& x_init, const Eigen::MatrixXd& P_init,
                 const Eigen::MatrixXd& F, const Eigen::MatrixXd& H, const Eigen::MatrixXd& Q,
                 const Eigen::MatrixXd& R);

    void predict();

    void update(const Eigen::VectorXd& z);

    Eigen::VectorXd getState() const {return x_;}

    Eigen::MatrixXd getCovariance() const {return P_;}





private:
    Eigen::VectorXd x_;   // state estimate
    Eigen::MatrixXd P_;   // covariance
    Eigen::MatrixXd F_;   // motion model
    Eigen::MatrixXd H_;   // measurement model
    Eigen::MatrixXd Q_;   // process noise
    Eigen::MatrixXd R_;   // measurement noise
};