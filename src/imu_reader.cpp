#include "vio/imu_reader.hpp"

#include <fstream>     //file streams
#include <sstream>     //string streams
#include <stdexcept>
#include <string>
#include <cstdint>
#include <iostream>


ImuReader::ImuReader(const std::string& csv_path){
    std::ifstream file(csv_path);
    if(!file.is_open()){
        throw std::runtime_error("Failed to open IMU CSV: " + csv_path);
    }

    std::string line;

    while(std::getline(file, line)){
        ImuMeasurement measurement;
        if(line[0] == '#'){
            continue;
        }

        std::stringstream ss(line);
        std::string token;
        int count = 0;
        Eigen::Vector3d gyro_;
        Eigen::Vector3d accel_;
        while(std::getline(ss, token, ',')){
            if(count == 0){
                measurement.timestamp_ns = std::stoull(token);
            } else if(count > 0 && count < 4){
                gyro_(count-1) = std::stod(token);
                if(count == 3){
                    measurement.gyro = gyro_;
                }
            } else {
                accel_(count - 4) = std::stod(token);
                if(count == 6){
                    measurement.accel = accel_;
                }
            }
            count++;

        }
        measurements_.push_back(measurement);
    }

}