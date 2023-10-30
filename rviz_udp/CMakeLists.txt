#1029 TODO
cmake_minimum_required(VERSION 2.8.3)
project(rviz)

set(CMAKE_BUILD_TYPE "Release")#Release Debug
set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

set(Eigen3_DIR ${CMAKE_SOURCE_DIR}/../env/eigen-3.4.0-install-x86/share/eigen3/cmake)
find_package(Eigen3 REQUIRED)
MESSAGE(STATUS "EIGEN3_INCLUDE_DIR------------${EIGEN3_INCLUDE_DIR}------------------")


find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        nav_msgs
        )

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ./../src/udp_helper.h
)

MESSAGE(STATUS "------------${catkin_LIBRARIES}------------------")
MESSAGE(STATUS "------------${catkin_INCLUDE_DIRS}------------------")

add_executable(rvizudp ./rviz_udp.cpp)
target_link_libraries(rvizudp  ${catkin_LIBRARIES})