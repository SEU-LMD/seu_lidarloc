cmake_minimum_required(VERSION 3.0)

project(data_check_plot)


set(sciplot_DIR ${CMAKE_SOURCE_DIR}/../../env/sciplot-install/share/sciplot)


find_package(sciplot)

set(Eigen3_DIR ${CMAKE_SOURCE_DIR}/../../env/eigen-3.4.0-install-x86/share/eigen3/cmake)
find_package(Eigen3 REQUIRED)
MESSAGE(STATUS "EIGEN3_INCLUDE_DIR------------${EIGEN3_INCLUDE_DIR}------------------")



include_directories(

        ${EIGEN3_INCLUDE_DIR}

)


add_executable(data_check_plot data_check_plot.cpp)

target_link_libraries(data_check_plot sciplot::sciplot)


