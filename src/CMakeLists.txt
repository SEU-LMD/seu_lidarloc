cmake_minimum_required(VERSION 2.8.3)
project(lio_sam_6axis)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")


find_package(Boost REQUIRED COMPONENTS timer)

find_package(catkin REQUIRED COMPONENTS
        tf
        roscpp
        rospy
        pcl_conversions
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        message_generation
        visualization_msgs
        rosbag
        )


set(Eigen3_DIR ${CMAKE_SOURCE_DIR}/../env/eigen-3.4.0-install-x86/share/eigen3/cmake)
find_package(Eigen3 REQUIRED)
MESSAGE(STATUS "EIGEN3_INCLUDE_DIR------------${EIGEN3_INCLUDE_DIR}------------------")

set(GTSAM_DIR ${CMAKE_SOURCE_DIR}/../env/gtsam-4.0.3-install/lib/cmake/GTSAM)
find_package(GTSAM REQUIRED)


set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/../env/opencv-3.4.16-install/share/OpenCV)
find_package(OpenCV REQUIRED)
       

set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
# find_package(PCL 1.10 REQUIRED)
find_package(PCL REQUIRED)



add_message_files(
        DIRECTORY msg
        FILES
        cloud_info.msg
)

add_service_files(
        DIRECTORY srv
        FILES
        save_map.srv
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
        nav_msgs
        sensor_msgs
)
catkin_package(
        INCLUDE_DIRS include
        DEPENDS PCL GTSAM

        CATKIN_DEPENDS
        std_msgs
        nav_msgs
        geometry_msgs
        sensor_msgs
        message_runtime
        message_generation
        visualization_msgs
)

add_subdirectory(./ThirdParty/GeographicLib/)



include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ./ThirdParty/GeographicLib/
)

MESSAGE(STATUS "------------${EIGEN3_INCLUDE_DIR}------------------")
MESSAGE(STATUS "------------${GTSAM_INCLUDE_DIR}------------------")
MESSAGE(STATUS "------------${PCL_INCLUDE_DIRS}------------------")
MESSAGE(STATUS "------------${PCL_LIBRARY_DIRS}------------------")

###########
## Build ##
###########

# Gps odometry
add_executable(${PROJECT_NAME}_gpsOdometry src/simpleGpsOdom.cpp)
target_link_libraries(${PROJECT_NAME}_gpsOdometry ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} libGeographic)

# # Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Feature Association
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmizationGps.cpp src/dataSaver.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam  libGeographic)

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )

#  add_executable(main main.cpp)
#  target_link_libraries(main gtsam)
