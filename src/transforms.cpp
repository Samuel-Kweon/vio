#include "vio/transforms.hpp"
#include <cmath>

namespace transforms {

Eigen::Matrix3d Rx(double theta_deg) {
    // Convert deg to rad: theta_deg * M_PI / 180.0
    double theta = theta_deg * M_PI / 180;

    Eigen::Matrix3d R;
    R <<    1,          0,           0,
            0, std::cos(theta), -std::sin(theta),
            0, std::sin(theta),  std::cos(theta);

    return R;
}

Eigen::Matrix3d Ry(double theta_deg) {
    double theta = theta_deg * M_PI / 180;

    Eigen::Matrix3d R;
    R <<     std::cos(theta),          0,     std::sin(theta),
                           0,          1,                   0,
            -std::sin(theta),          0,     std::cos(theta);
    
    return R;
}

Eigen::Matrix3d Rz(double theta_deg) {
    double theta = theta_deg * M_PI / 180;

    Eigen::Matrix3d R;
    R <<    std::cos(theta), -std::sin(theta),           0,
            std::sin(theta),  std::cos(theta),           0,
                          0,                0,           1;
    
    return R;
}

Eigen::Matrix4d makeTransform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    // YOUR CODE HERE
    // Hint: start with Eigen::Matrix4d::Identity(), then assign blocks
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;
    
    return T;
}

Eigen::Matrix4d invertTransform(const Eigen::Matrix4d& T) {
    // YOUR CODE HERE
    // Use the SE(3) shortcut: inverse rotation = R.transpose(), inverse translation = -R.transpose() * t
    // Extract R from T.block<3,3>(0,0), t from T.block<3,1>(0,3)
    // Then assemble using makeTransform
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d t = T.block<3,1>(0,3);
    Eigen::Matrix3d R_inv = R.transpose();
    Eigen::Vector3d t_inv = -R_inv * t;

    return makeTransform(R_inv, t_inv);
}

Eigen::Vector3d transformPoint(const Eigen::Matrix4d& T, const Eigen::Vector3d& p) {
    // YOUR CODE HERE
    // Make p homogeneous (append 1), multiply by T, drop the last component
    // Or equivalently: result = R * p + t where R and t are extracted from T
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d t = T.block<3,1>(0,3);
    return R*p+t;


}

} // namespace transforms