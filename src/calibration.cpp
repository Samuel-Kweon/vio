#include "vio/calibration.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>


namespace {

// Trim whitespace from both ends of a string
std::string trim(const std::string& s) {
    auto begin = s.find_first_not_of(" \t\n\r");  //space, tab, newline. returns the first char in s that's not in that set.
    if(begin == std::string::npos) return ""; //npos = not found or no position
    auto end = s.find_last_not_of("\t\n\r");
    return s.substr(begin, end - begin+1); //(start, length) +1 for inclusive 
}

// Extract a list of doubles from a bracketed string like "[1.0, 2.0, 3.0]"
std::vector<double> parseDoubleList(const std::string& bracketed) {
    std::vector<double> result;
    std::string inner = bracketed;

    //Remove brackets and any surrounding whitespace
    auto open = inner.find('[');
    auto close = inner.find(']');
    if(open == std::string::npos || close == std::string::npos){
        throw std::runtime_error("parseDoubleList: missing brackets in : " + bracketed);
    }
    inner = inner.substr(open + 1, open - close - 1); // -1 for excluding the bracketes

    //Strip out the commments (anything after #)
    auto comment = inner.find('#');
    if(comment != std::string::npos) inner = inner.substr(0, comment); //from 0 to where # starts. 

    // Split on commas
    std::stringstream ss(inner);
    std::string token;
    while(std::getline(ss, token, ',')){
        token = trim(token);
        if(!token.empty()) {
            result.push_back(std::stod(token));
        }
    }
    return result;

}

} //anonymous namespace


Calibration::Calibration(const std::string& mav0_path){
    loadCamera(mav0_path + "/cam0/sensor.yaml", cam0_);
}

void Calibration::loadCamera(const std::string& yaml_path, CameraCalibration& cam){
    std::ifstream file(yaml_path);
    if(!file.is_open()){
        throw std::runtime_error("Failed to open calibration file: " + yaml_path);
    }
    
    // Read the whole file into a string for easy multi-line parsing
    std::stringstream buffer;
    buffer << file.rdbuf();                // dump whole file into buffer
    std::string content = buffer.str();    // pull it out as a string, str() returns the contents of the stringstream as a std::string

    // Parse T_BS 
    // Find  "data: " inside the T_BS block, then collect the 16 numbers
    auto data_pos = content.find("data:");
    if(data_pos == std::string::npos){
        throw std::runtime_error("T_BS data not found in "  + yaml_path);
    }

    auto bracket_open = content.find('[', data_pos);     //search starts from data_pos
    auto bracket_close = content.find(']', bracket_open);
    if(bracket_open == std::string::npos || bracket_close == std::string::npos){
        throw std::runtime_error("T_BS brackets not found");
    }
    std::string t_bs_str = content.substr(bracket_open, bracket_close - bracket_open + 1);
    std::vector<double> t_bs = parseDoubleList(t_bs_str);
    if(t_bs.size() != 16){
        throw std::runtime_error("T_BS does not have 16 elements");
    }  
    // Fill the 4x4 matrix (row-major in YAML, row-major in Eigen)
    for (int r = 0; r < 4; ++r){
        for (int c = 0; c < 4; ++c){
            cam.T_BC(r,c) = t_bs[r * 4 + c];
        }
    } //----------------Below read over again---------------------

    // Parse simple scalar fields
    std::istringstream iss(content);
    std::string line;
    while (std::getline(iss, line)){
        std::string trimmed = trim(line);

        // Skip comments and blank lines
        if(trimmed.empty() || trimmed[0] == '#') continue;

        // Find colon separator
        auto colon = trimmed.find(':');
        if(colon == std::string::npos) continue;

        std::string key = trim(trimmed.substr(0, colon));
        std::string value = trim(trimmed.substr(colon + 1));

        // Strip inline commnets from value
        auto hash = value.find('#');
        if (hash != std::string::npos) value = trim(value.substr(0, hash));
        
        if (key == "rate_hz") {
            cam.rate_hz = std::stod(value);
        } 
        else if (key == "resolution"){
            auto nums = parseDoubleList(value);
            if (nums.size() >= 2){
                cam.width = static_cast<int>(nums[0]);
                cam.height = static_cast<int>(nums[1]);
            }
        }
        else if (key == "camera_model"){
            cam.camera_model = value;
        }
        else if (key == "intrinsics"){
            auto nums = parseDoubleList(value);
            if(nums.size() != 4){
                throw std::runtime_error("intrinsics should have 4 values");
            }
            //[fx, fy, cx, cy]
            cam.K(0,0) = nums[0];
            cam.K(1,1) = nums[1];
            cam.K(0,2) = nums[2];
            cam.K(1,2) = nums[3];
            cam.K(2,2) = 1.0;
        }
        else if (key == "distortion_model"){
            cam.distortion_model = value;
        }
        else if (key == "distortion_coefficients"){
            auto nums = parseDoubleList(value);
            cam.dist_coeffs.resize(nums.size());
            for (size_t i = 0; i < nums.size(); i++){
                cam.dist_coeffs(i) = nums[i];
            }
        }

    }
}











