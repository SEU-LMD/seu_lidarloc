# Install script for directory: /home/today/thirtyparty/pcl-1.10.0/sample_consensus

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_sample_consensus" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_sample_consensus.so.1.10.0"
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_sample_consensus.so.1.10"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so.1.10"
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_sample_consensus" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so"
         RPATH "/usr/local/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_sample_consensus.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so"
         OLD_RPATH "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib:"
         NEW_RPATH "/usr/local/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_sample_consensus.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_sample_consensus" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/sample_consensus/pcl_sample_consensus-1.10.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_sample_consensus" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/sample_consensus" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/boost.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/eigen.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/lmeds.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/method_types.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/mlesac.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/model_types.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/msac.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/ransac.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/rmsac.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/rransac.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/prosac.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model_circle.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model_circle3d.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model_cylinder.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model_cone.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model_line.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model_stick.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model_normal_parallel_plane.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model_normal_plane.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model_normal_sphere.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model_parallel_line.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model_parallel_plane.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model_perpendicular_plane.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model_plane.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model_registration.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model_registration_2d.h"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/sac_model_sphere.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_sample_consensus" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/sample_consensus/impl" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/lmeds.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/mlesac.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/msac.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/ransac.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/rmsac.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/rransac.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/prosac.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/sac_model_circle.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/sac_model_circle3d.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/sac_model_cylinder.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/sac_model_cone.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/sac_model_line.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/sac_model_stick.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/sac_model_normal_parallel_plane.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/sac_model_normal_plane.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/sac_model_normal_sphere.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/sac_model_parallel_line.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/sac_model_parallel_plane.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/sac_model_plane.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/sac_model_registration.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/sac_model_registration_2d.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/sample_consensus/include/pcl/sample_consensus/impl/sac_model_sphere.hpp"
    )
endif()

