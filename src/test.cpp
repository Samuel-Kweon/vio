#include "vio/imu_reader.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>


ImuReader::ImuReader(const std::string& csv_path){
    std::ifstream file(csv_path);
    if(!file.is_open()){
        throw std::runtime_error("Failed to open the file" + csv_path);
    }

    std::string line;

    while(std::getline(file, line)){
        if(line[0] == #){
            continue;
        }
        std::stringstream ss = line;
        std::string element;
        int count = 0;
        while(std::getline(ss, element, ',')){
            if(count == 0){
                measurements_.timestamp_ns = std::stod(element);
            }
            else if (count == 1){

            }
        }

    }
}