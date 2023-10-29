# Install script for directory: /home/today/thirtyparty/pcl-1.10.0/keypoints

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_keypoints" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_keypoints.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_keypoints.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib:/home/today/thirtyparty/flann-1.9.2-install/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_keypoints.so.1.10.0"
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_keypoints.so.1.10"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_keypoints.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_keypoints.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib:/home/today/thirtyparty/flann-1.9.2-install/lib:"
           NEW_RPATH "/usr/local/lib:/home/today/thirtyparty/flann-1.9.2-install/lib")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_keypoints" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_keypoints.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_keypoints.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_keypoints.so"
         RPATH "/usr/local/lib:/home/today/thirtyparty/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_keypoints.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_keypoints.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_keypoints.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_keypoints.so"
         OLD_RPATH "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib:/home/today/thirtyparty/flann-1.9.2-install/lib:"
         NEW_RPATH "/usr/local/lib:/home/today/thirtyparty/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_keypoints.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_keypoints" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/keypoints/pcl_keypoints-1.10.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_keypoints" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/keypoints" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/keypoint.h"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/narf_keypoint.h"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/sift_keypoint.h"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/uniform_sampling.h"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/smoothed_surfaces_keypoint.h"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/agast_2d.h"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/harris_2d.h"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/harris_3d.h"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/harris_6d.h"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/susan.h"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/iss_3d.h"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/brisk_2d.h"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/trajkovic_2d.h"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/trajkovic_3d.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_keypoints" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/keypoints/impl" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/impl/keypoint.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/impl/sift_keypoint.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/impl/smoothed_surfaces_keypoint.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/impl/agast_2d.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/impl/harris_2d.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/impl/harris_3d.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/impl/harris_6d.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/impl/susan.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/impl/iss_3d.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/impl/brisk_2d.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/impl/trajkovic_2d.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/keypoints/include/pcl/keypoints/impl/trajkovic_3d.hpp"
    )
endif()

