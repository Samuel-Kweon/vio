#include "vio/camera_reader.hpp"
#include <fstream>
#include <sstream>
#include <iostream>




CameraReader::CameraReader(const std::string& cam_path){
    path_ = cam_path;
    std::ifstream file(cam_path + "/data.csv");
    if(!file.is_open()){
        throw std::runtime_error("Camera Data file is not opening" + cam_path);
    }

    std::string line;
    while(std::getline(file, line)){
        CameraData data;
        if(line[0] == '#'){
            continue;
        }
        std::stringstream ss(line);
        std::string token;
        int count = 0;
        while(std::getline(ss, token, ',')){
            if(count == 0){
                data.timestamp_ns = std::stoull(token);
            } else {
                token.erase(token.find_last_not_of("\t\n\r") + 1);
                data.filename = token;
            }
            count++;
        }

        dataset_.push_back(data);

    }

}

cv::Mat CameraReader::loadImage(int frame){

    std::string filename = dataset_[frame].filename;

    cv::Mat img = cv::imread(path_ + "/data/" + filename, cv::IMREAD_GRAYSCALE);
    if(img.empty()){
        throw std::runtime_error("Image not found: " + filename + '\n');
    }

    return img;
}