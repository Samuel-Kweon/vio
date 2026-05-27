#pragma once
#include <Eigen/Dense>

namespace transforms {
    
    // 3D rotation matrices around each axis
    Eigen::Matrix3d Rx(double theta_deg);
    Eigen::Matrix3d Ry(double theta_deg);
    Eigen::Matrix3d Rz(double theta_deg);

    // Build a 4x4 SE(3) transform from rotation and translation
    Eigen::Matrix4d makeTransform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

    // Invert a SE(3) transform using the closed-form shortcut
    Eigen::Matrix4d invertTransform(const Eigen::Matrix4d& T);

    // Apply a 4x4 transform to a 3D point
    Eigen::Vector3d transformPoint(const Eigen::Matrix4d&T, const Eigen::Vector3d& p);
}














