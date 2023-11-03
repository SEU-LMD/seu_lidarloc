cmake_minimum_required(VERSION 2.8.3)
project(map_engine)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

add_definitions(-w)


#find_package(Boost REQUIRED COMPONENTS timer)

set(Eigen3_DIR ${CMAKE_SOURCE_DIR}/../../env/eigen-3.4.0-install-x86/share/eigen3/cmake)
find_package(Eigen3 REQUIRED)
MESSAGE(STATUS "EIGEN3_INCLUDE_DIR------------${EIGEN3_INCLUDE_DIR}------------------")


set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package(PCL REQUIRED)


set(YAML_CPP_DIR "${CMAKE_SOURCE_DIR}/../../env/yaml-cpp-install/lib/cmake/yaml-cpp")
find_package(YAML_CPP REQUIRED) 
set(YAML_LIB "${CMAKE_SOURCE_DIR}/../../env/yaml-cpp-install/lib/libyaml-cpp.a")

#use for easylog marco
add_definitions(-DELPP_THREAD_SAFE)
#add_definitions(-DELPP_FORCE_USE_STD_THREAD)
add_definitions(-DELPP_NO_DEFAULT_LOG_FILE)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        )


        include_directories(
        ../include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ../ThirdParty/easylog-9.89/src/
        ${YAML_CPP_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
                ../utils
)

MESSAGE(STATUS "------------${EIGEN3_INCLUDE_DIR}------------------")
MESSAGE(STATUS "------------${PCL_INCLUDE_DIRS}------------------")
MESSAGE(STATUS "------------${PCL_LIBRARY_DIRS}------------------")

###########
## Build ##
###########

#mapping related executabel
add_executable(serialize map_serialize.cpp)
target_link_libraries(serialize ${PCL_LIBRARIES} ${YAML_LIB})

#add_executable(mapmanager ./mapmanager.cpp)
#target_link_libraries(mapmanager ${PCL_LIBRARIES} ${YAML_LIB} ${catkin_LIBRARIES})
