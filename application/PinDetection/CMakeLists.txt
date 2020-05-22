cmake_minimum_required(VERSION 3.5)

project(PIN_DETECTION)

set (CMAKE_CXX_STANDARD 11)

find_package( OpenCV REQUIRED )

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(/opt/ros/kinetic/include/opencv-3.3.1-dev)

add_executable(
    pin_detection
    Source.cpp
    PinDetection.cpp 
    ../../ImageCluster/ImageCluster.cpp
    ../../ImageCluster/Cluster.cpp)

target_link_libraries( pin_detection ${OpenCV_LIBS} )
