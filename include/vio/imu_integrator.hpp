#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "vio/imu_reader.hpp"
 
struct ImuState {
    uint64_t timestamp_ns;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d bias_accel;
    Eigen::Vector3d bias_gyro;

};

class ImuIntegrator {
public:
    ImuIntegrator(const ImuState& initial_state, const Eigen::Vector3d& gravity_w);

    void integrate(const ImuMeasurement& m);

    const ImuState& state() const {return state_;}

private:
    ImuState state_;
    Eigen::Vector3d gravity_w_;
    bool first_measurement_ = true;
};