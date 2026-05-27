#pragma once
#include <Eigen/Dense>
#include <functional>

class EKF{
public:

    using MotionFn        = std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>;
    using MotionJacobian  = std::function<Eigen::MatrixXd(const Eigen::MatrixXd&, const Eigen::VectorXd&)>;
    using MeasureFn       = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
    using MeasureJacobian = std::function<Eigen::MatrixXd(const Eigen::MatrixXd&)>;   

    EKF(const Eigen::VectorXd& x_init, const Eigen::MatrixXd& P_init, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, MotionFn f, MotionJacobian F, MeasureFn h, MeasureJacobian H);

    void predict(const Eigen::VectorXd& u);

    void update(const Eigen::VectorXd& z);

    Eigen::VectorXd getState() const { return x_; }
    Eigen::MatrixXd getCovariance() const { return P_; }

private:
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    MotionFn f_;
    MotionJacobian F_;
    MeasureFn h_;
    MeasureJacobian H_;

};

