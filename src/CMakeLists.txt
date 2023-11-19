cmake_minimum_required(VERSION 2.8.3)
project(seu_lidarloc)

set(CMAKE_BUILD_TYPE "Release")#Release Debug
set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

add_definitions(-w)

find_package(Boost REQUIRED COMPONENTS timer)

#according to different platform to choose communication tool , ros or mdc
set(COMPILE_MODE "X86" CACHE STRING "choose the platform")# X86 or MDC
#set(COMPILE_MODE "MDC" CACHE STRING "choose the platform")# X86 or MDC
if( ${COMPILE_MODE} STREQUAL "X86")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/X86.cmake)
elseif( ${COMPILE_MODE} STREQUAL "MDC")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/MDC.cmake)
endif()
message("Compile mode: ${COMPILE_MODE}")

set(Eigen3_DIR ${CMAKE_SOURCE_DIR}/../env/eigen-3.4.0-install-x86/share/eigen3/cmake)
find_package(Eigen3 REQUIRED)
MESSAGE(STATUS "EIGEN3_INCLUDE_DIR------------${EIGEN3_INCLUDE_DIR}------------------")

set(GTSAM_DIR ${CMAKE_SOURCE_DIR}/../env/gtsam-4.0.3-install/lib/cmake/GTSAM)
find_package(GTSAM REQUIRED)

set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/../env/opencv-3.4.16-install/share/OpenCV)
find_package(OpenCV REQUIRED)

set(PCL_DIR ${CMAKE_SOURCE_DIR}/../env/pcl-1.10.0-install/share/pcl-1.10)
find_package(PCL REQUIRED)
MESSAGE("-----------PCL_ROOT:${PCL_ROOT}-----------")

set(YAML_CPP_DIR "${CMAKE_SOURCE_DIR}/../env/yaml-cpp-install/lib/cmake/yaml-cpp")
find_package(YAML_CPP REQUIRED) 
set(YAML_LIB "${CMAKE_SOURCE_DIR}/../env/yaml-cpp-install/lib/libyaml-cpp.a")

set(FLANN_DIR ${CMAKE_SOURCE_DIR}/../env/flann-1.9.2-install/lib/cmake/flann)
find_package(FLANN REQUIRED)
set(FLANN_DIRS ${CMAKE_SOURCE_DIR}/../env/flann-1.9.2-install/include)



set(PCL_LIBRARIES
        ${CMAKE_SOURCE_DIR}/../env/pcl-1.10.0-install/lib/libpcl_common.so.1.10
        ${CMAKE_SOURCE_DIR}/../env/pcl-1.10.0-install/lib/libpcl_io.so.1.10
        ${CMAKE_SOURCE_DIR}/../env/pcl-1.10.0-install/lib/libpcl_io_ply.so.1.10)

#use for easylog marco
add_definitions(-DELPP_THREAD_SAFE)
add_definitions(-DELPP_NO_DEFAULT_LOG_FILE)
#add_definitions(-DELPP_FORCE_USE_STD_THREAD)

add_subdirectory(./ThirdParty/GeographicLib/)

include_directories(
        ./
        ./src
        ./front_end
        ${Communication_DIR}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ./ThirdParty/GeographicLib/
        ./ThirdParty/easylog-9.89/src/
        ${YAML_CPP_INCLUDE_DIR}
        ${FLANN_DIRS}
        ./udp_helper.h
)

MESSAGE(STATUS "------------${EIGEN3_INCLUDE_DIR}------------------")
MESSAGE(STATUS "------------${GTSAM_INCLUDE_DIR}------------------")
MESSAGE(STATUS "------------${PCL_INCLUDE_DIRS}------------------")
MESSAGE(STATUS "PCL_LIBRARIES:------------${PCL_LIBRARY_DIRS}------------------")
FILE(WRITE cur_pwd.txt ${CMAKE_SOURCE_DIR}) # add by sy

set(ABS_CURRENT_SOURCE_PATH ${CMAKE_SOURCE_DIR})

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/config/abs_current_path.h.ini
        ${CMAKE_SOURCE_DIR}/config/abs_current_path.h)

###########
## Build ##
############
#add_executable(mapping app_mapping/mapping.cpp)
#target_link_libraries(mapping ${Communication_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} libGeographic gtsam ${YAML_LIB})#YAML_CPP_LIBRARIES

add_executable(loc app_loc/loc.cpp)
target_link_libraries(loc ${Communication_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} libGeographic  gtsam ${YAML_LIB})#YAML_CPP_LIBRARIES

#add_executable(dr_calib app_dr_calib/dr_calib.cpp)
#target_link_libraries(dr_calib ${Communication_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} libGeographic gtsam ${YAML_LIB})#YAML_CPP_LIBRARIES

#add_executable(data_check app_datacheck/datacheck.cpp)
#target_link_libraries(data_check ${Communication_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} libGeographic gtsam ${YAML_LIB})#YAML_CPP_LIBRARIES