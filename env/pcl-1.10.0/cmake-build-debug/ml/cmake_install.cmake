# Install script for directory: /home/today/thirtyparty/pcl-1.10.0/ml

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_ml.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_ml.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_ml.so.1.10.0"
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_ml.so.1.10"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_ml.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_ml.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib:"
           NEW_RPATH "/usr/local/lib")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_ml.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_ml.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_ml.so"
         RPATH "/usr/local/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_ml.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_ml.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_ml.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_ml.so"
         OLD_RPATH "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib:"
         NEW_RPATH "/usr/local/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_ml.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/ml/pcl_ml-1.10.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/ml" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/branch_estimator.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/feature_handler.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/multi_channel_2d_comparison_feature.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/multi_channel_2d_comparison_feature_handler.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/multi_channel_2d_data_set.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/multiple_data_2d_example_index.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/point_xy_32i.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/point_xy_32f.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/regression_variance_stats_estimator.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/stats_estimator.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/densecrf.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/pairwise_potential.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/permutohedral.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/svm_wrapper.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/svm.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/kmeans.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/ml/dt" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/dt/decision_forest.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/dt/decision_forest_evaluator.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/dt/decision_forest_trainer.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/dt/decision_tree.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/dt/decision_tree_evaluator.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/dt/decision_tree_trainer.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/dt/decision_tree_data_provider.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/ml/ferns" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/ferns/fern.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/ferns/fern_evaluator.h"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/ferns/fern_trainer.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/ml/impl/dt" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/impl/dt/decision_forest_evaluator.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/impl/dt/decision_forest_trainer.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/impl/dt/decision_tree_evaluator.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/impl/dt/decision_tree_trainer.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/ml/impl/ferns" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/impl/ferns/fern_evaluator.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/impl/ferns/fern_trainer.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_ml" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/ml/impl/svm" TYPE FILE FILES "/home/today/thirtyparty/pcl-1.10.0/ml/include/pcl/ml/impl/svm/svm_wrapper.hpp")
endif()

