#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <filesystem>


struct CameraData {
    uint64_t timestamp_ns;
    std::string filename;

};

class CameraReader {
public:
    explicit CameraReader(const std::string& csv_path);

    size_t size() const{
        return dataset_.size();
    }

    uint64_t timestamp(int frame) const {
        return dataset_[frame].timestamp_ns;
    }

    cv::Mat loadImage(int frame);



private:
    std::vector<CameraData> dataset_;
    std::string path_;

};