#pragma once
#include <string>
#include <Eigen/Dense>

struct CameraCalibration {
    Eigen::Matrix4d T_BC;
    Eigen::Matrix3d K;
    Eigen::VectorXd dist_coeffs;
    int width = 0;
    int height = 0;
    double rate_hz = 0.0;
    std::string distortion_model;
    std::string camera_model;
};

class Calibration {
public:
    explicit Calibration(const std::string& mav0_path);
    const CameraCalibration cam0() const {return cam0_;};
private:
    void loadCamera(const std::string& yaml_path, CameraCalibration& cam);
    CameraCalibration cam0_;
};
