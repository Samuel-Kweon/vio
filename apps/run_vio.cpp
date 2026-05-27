#include <iostream>
#include <vio/transforms.hpp>
#include <vio/kalman.hpp>
#include <vio/ekf.hpp>

int main() {
    std::cout << "VIO project skeleton - hello from Session 11\n";

    Eigen::MatrixXd R = transforms::Rz(90.0);
    Eigen::Vector3d t(2, 0, 5);
    Eigen::Matrix4d T = transforms::makeTransform(R, t);
    std::cout << "T_world_from_camera =\n" << T << '\n';
    
    return 0;
}