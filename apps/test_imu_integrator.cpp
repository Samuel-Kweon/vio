#include "vio/imu_reader.hpp"
#include "vio/imu_integrator.hpp"
#include <iostream>
#include <iomanip>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_imu_data.csv>\n";
        return 1;
    }

    ImuReader reader(argv[1]);
    std::cout << "Loaded " << reader.size() << " IMU measurements\n";

    // === Initial state ===
    // We're "cheating" here by using approximate values from EuRoC's ground truth at t=0.
    // In a real system you'd estimate this from initial stationary IMU readings.
    ImuState initial_state;
    initial_state.position    = Eigen::Vector3d(4.688319, -1.786938, 0.783338);
    initial_state.velocity    = Eigen::Vector3d(-0.027876,0.033207,0.800006);
    initial_state.orientation = Eigen::Quaterniond(0.534108, -0.153029, -0.827383, -0.082152);  // (w, x, y, z)
    initial_state.bias_gyro  = Eigen::Vector3d(-0.003172,0.021267,0.078502);
    initial_state.bias_accel   = Eigen::Vector3d(-0.025266,0.136696,0.075593);
    initial_state.timestamp_ns = 1403636580838555648;

    // Gravity in world frame (z-up convention)
    Eigen::Vector3d gravity_w(0, 0, -9.81);

    ImuIntegrator integrator(initial_state, gravity_w);

    // === Integrate ===
    std::cout << std::fixed << std::setprecision(3);
    
    for (size_t i = 0; i < reader.size(); ++i) {
        integrator.integrate(reader.at(i));
        
        // Print state every 1000 measurements (every 5 seconds at 200 Hz)
        if (i % 1000 == 0) {
            const auto& s = integrator.state();
            double t_s = (s.timestamp_ns - reader.at(0).timestamp_ns) * 1e-9;
            std::cout << "t=" << std::setw(7) << t_s << "s  "
                      << "pos=[" << std::setw(8) << s.position(0) << ", "
                      <<            std::setw(8) << s.position(1) << ", "
                      <<            std::setw(8) << s.position(2) << "]  "
                      << "|v|=" << s.velocity.norm() << "\n";
        }
    }

    const auto& final = integrator.state();
    std::cout << "\nFinal state:\n";
    std::cout << "  position: " << final.position.transpose() << "\n";
    std::cout << "  velocity: " << final.velocity.transpose() << "\n";
    std::cout << "  |v|:      " << final.velocity.norm() << " m/s\n";

    return 0;
}