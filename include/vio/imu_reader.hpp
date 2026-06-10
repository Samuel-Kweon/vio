#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <Eigen/Dense>

struct ImuMeasurement {
    uint64_t timestamp_ns;      //nanoseconds since epoch
    Eigen::Vector3d gyro;       //angular velocity [rad/s]
    Eigen::Vector3d accel;      //linear acceleration [m/s^2]

};

class ImuReader{
public:
    explicit ImuReader(const std::string& csv_path);

    size_t size() const {
        return measurements_.size();
    }

    const ImuMeasurement& at(size_t i) const {
        return measurements_.at(i);
    }

    const std::vector<ImuMeasurement>& all() const {
        return measurements_;
    }

private:
    std::vector<ImuMeasurement> measurements_;
};