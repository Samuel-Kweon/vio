#include "vio/camera_reader.hpp"
#include <iostream>
#include <iomanip>

int main(int argc, char** argv){
    if(argc < 2){
        std::cerr << "Error: missing argument.\n";
        std::cerr << "Usage:" << argv[0] << " <path_to_cam0_data.csv>\n";
        return 1;
    }

    CameraReader reader(argv[1]);

    if(reader.size() == 0){
        std::cerr << "No data loaded!\n";
        return 1;
    }

    auto size = reader.size();
    auto first = reader.timestamp(0);
    auto last = reader.timestamp(reader.size()-1);
    double dt_seconds = (last - first)/1e9;
    std::cout << "Frames: " << size << '\n';
    std::cout << "First timestamp: " << first << '\n';
    std::cout << "Last timestamp: " << last << '\n';
    std::cout << "Average framerate: " << size/dt_seconds << "hz\n";

    cv::Mat img = reader.loadImage(0);
    cv::imshow("First image", img);
    cv::waitKey(0);


    return 0;
}