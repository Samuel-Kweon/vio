#include "vio/imu_reader.hpp"
#include <iostream>
#include <iomanip>

int main(int argc, char** argv) {
    if(argc < 2){
        std::cerr << "Usage: " << argv[0] << " <path_to_imu_data.csv>\n";
        return 1;
    }

    ImuReader reader(argv[1]);

    std::cout << "Loaded: " << reader.size() << " IMU measurements\n";

    if(reader.size() == 0){
        std::cerr << "No measurements loaded!\n";
        return 1;
    }

    const auto& first = reader.at(0);
    const auto& last = reader.at(reader.size() - 1);

    std::cout << std::fixed << std::setprecision(6);


    std::cout << "\nFirst reading:\n";
    std::cout << "  timestamp: " << first.timestamp_ns << " ns\n";
    std::cout << "  gyro:      " << first.gyro.transpose() << "\n";
    std::cout << "  accel:     " << first.accel.transpose() << "\n";

    std::cout << "\nLast reading:\n";
    std::cout << "  timestamp: " << last.timestamp_ns << " ns\n";
    std::cout << "  gyro:      " << last.gyro.transpose() << "\n";
    std::cout << "  accel:     " << last.accel.transpose() << "\n";

    double duration_s = (last.timestamp_ns - first.timestamp_ns) * 1e-9;
    double rate_hz = (reader.size() - 1) / duration_s;
    std::cout << "\nDuration: " << duration_s << '\n';
    std::cout << "Averate rate: " << rate_hz << '\n';

    return 0;

}