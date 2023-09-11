cmake_minimum_required(VERSION 2.8.3)
project(seu_lidarloc)

set(CMAKE_BUILD_TYPE "Debug")#Release Debug
set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

add_definitions(-w)

find_package(Boost REQUIRED COMPONENTS timer)

#according to different platform to choose communication tool , ros or mdc
set(COMPILE_MODE "X86" CACHE STRING "choose the platform")# X86 or MDC
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
       

set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package(PCL REQUIRED)

set(YAML_CPP_DIR "${CMAKE_SOURCE_DIR}/../env/yaml-cpp-install/lib/cmake/yaml-cpp")
find_package(YAML_CPP REQUIRED) 
set(YAML_LIB "${CMAKE_SOURCE_DIR}/../env/yaml-cpp-install/lib/libyaml-cpp.a")

#use for easylog marco
add_definitions(-DELPP_THREAD_SAFE)
add_definitions(-DELPP_NO_DEFAULT_LOG_FILE)
#add_definitions(-DELPP_FORCE_USE_STD_THREAD)

add_subdirectory(./ThirdParty/GeographicLib/)

include_directories(
        ./
        ./src
        ${Communication_DIR}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ./ThirdParty/GeographicLib/
        ./ThirdParty/easylog-9.89/src/
        ${YAML_CPP_INCLUDE_DIR}
)

MESSAGE(STATUS "------------${EIGEN3_INCLUDE_DIR}------------------")
MESSAGE(STATUS "------------${GTSAM_INCLUDE_DIR}------------------")
MESSAGE(STATUS "------------${PCL_INCLUDE_DIRS}------------------")
MESSAGE(STATUS "------------${PCL_LIBRARY_DIRS}------------------")

###########
## Build ##
###########
add_executable(mapping app_mapping/mapping.cpp)
target_link_libraries(mapping ${Communication_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} libGeographic gtsam ${YAML_LIB})#YAML_CPP_LIBRARIES

add_executable(loc app_loc/loc.cpp)
target_link_libraries(loc ${Communication_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} libGeographic  gtsam ${YAML_LIB})#YAML_CPP_LIBRARIES


## # # Range Image Projection
#add_executable(imageProjection src/imageProjection_gnss.cpp)
##add_dependencies(imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(imageProjection ${Communication_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} libGeographic  ${YAML_LIB})#YAML_CPP_LIBRARIES
##
### Feature Association
#add_executable(featureExtraction src/featureExtraction.cpp)
##add_dependencies(featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(featureExtraction ${Communication_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}  ${YAML_LIB})
#
## Mapping Optimization
#add_executable(mapOptmization src/mapOptmizationGps.cpp )
##add_dependencies(mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_compile_options(mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
#target_link_libraries(mapOptmization ${Communication_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}  gtsam  libGeographic  ${YAML_LIB})
#
## Prior Map based localization
#add_executable(priorMapLoc src/priorMapLoc.cpp )
##add_dependencies(priorMapLoc ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
##target_compile_options(priorMapLoc PRIVATE ${OpenMP_CXX_FLAGS})
#target_link_libraries(priorMapLoc ${Communication_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}  gtsam  libGeographic  ${YAML_LIB})

# IMU Preintegrations
#add_executable(imuPreintegration src/imuPreintegration.cpp)
#target_link_libraries(imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam  ${YAML_LIB})

