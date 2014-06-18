cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(connect_kinect)

find_package( catkin REQUIRED COMPONENTS roscpp )
find_package( OpenCV REQUIRED )
FIND_PACKAGE(OpenCV REQUIRED)
find_package(Boost REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(connect_kinect src/connect_kinect.cpp)


target_link_libraries (connect_kinect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} /opt/ros/hydro/lib/libcv_bridge.so)
