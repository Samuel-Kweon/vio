#include "vio/calibration.hpp"
#include <iostream>
#include <iomanip>

int main(int argc, char** argv){
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_mav0>\n";
        return 1;
    }

    Calibration calib(argv[1]);
    const auto& cam = calib.cam0();

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=== cam0 calibration ===\n\n";
    std::cout << "Camera model:     " << cam.camera_model     << "\n";
    std::cout << "Distortion model: " << cam.distortion_model << "\n";
    std::cout << "Resolution:       " << cam.width << " x " << cam.height << "\n";
    std::cout << "Rate:             " << cam.rate_hz << " Hz\n";
    std::cout << "\nIntrinsic matrix K:\n" << cam.K << "\n";
    std::cout << "\nDistortion coefficients: " << cam.dist_coeffs.transpose() << "\n";
    std::cout << "\nT_BC (camera-to-body):\n" << cam.T_BC << "\n";

    // Sanity check: extract rotation and translation
    Eigen::Matrix3d R_BC = cam.T_BC.block<3, 3>(0, 0);
    Eigen::Vector3d t_BC = cam.T_BC.block<3, 1>(0, 3);
    std::cout << "\nTranslation (camera origin in body frame): " << t_BC.transpose() << "\n";
    std::cout << "Translation magnitude: " << t_BC.norm() << " m\n";

    // Verify R_BC is a valid rotation: R^T R should be identity, det should be +1
    Eigen::Matrix3d RtR = R_BC.transpose() * R_BC;
    double det = R_BC.determinant();
    std::cout << "\nR^T R (should be identity):\n" << RtR << "\n";
    std::cout << "det(R_BC) (should be +1.0): " << det << "\n";

    return 0;
}