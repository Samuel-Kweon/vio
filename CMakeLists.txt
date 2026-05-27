cmake_minimum_required(VERSION 3.10)
project(vio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_library(vio_core src/ekf.cpp src/kalman.cpp src/transforms.cpp)

target_include_directories(vio_core PUBLIC include)
target_link_libraries(vio_core PUBLIC Eigen3::Eigen ${OpenCV_LIBS})

add_executable(run_vio apps/run_vio.cpp)
target_link_libraries(run_vio vio_core)

add_executable(test_transforms tests/test_transforms.cpp)
target_link_libraries(test_transforms vio_core)

