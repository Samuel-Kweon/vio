#include <vio/transforms.hpp>
#include <iostream>

int main() {
    using namespace transforms;

    // Build a transform: camera at (2, 0, 5), rotated 90° around z
    Eigen::Matrix3d R = Rz(90);
    Eigen::Vector3d t(2, 0, 5);
    Eigen::Matrix4d T = makeTransform(R, t);

    std::cout << "T =\n" << T << "\n\n";

    // Transform a point
    Eigen::Vector3d p_camera(1, 0, 0);
    Eigen::Vector3d p_world = transformPoint(T, p_camera);
    std::cout << "Point [1, 0, 0] in camera frame -> world: " << p_world.transpose() << "\n";
    std::cout << "Should be (2, 1, 5)\n\n";

    // Round-trip test
    Eigen::Matrix4d T_inv = invertTransform(T);
    Eigen::Vector3d p_recovered = transformPoint(T_inv, p_world);
    std::cout << "Round trip recovered: " << p_recovered.transpose() << "\n";
    std::cout << "Should be (1, 0, 0)\n\n";

    // Verify T * T_inv = I
    Eigen::Matrix4d should_be_identity = T * T_inv;
    std::cout << "T * T_inv =\n" << should_be_identity << "\n";
    std::cout << "Should be identity matrix\n";

    return 0;
}