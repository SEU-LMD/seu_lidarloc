# Install script for directory: /home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_io" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io_ply.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io_ply.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib/libpcl_io_ply.so.1.10.0"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib/libpcl_io_ply.so.1.10"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io_ply.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io_ply.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
           NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_io" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io_ply.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io_ply.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io_ply.so"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib/libpcl_io_ply.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io_ply.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io_ply.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io_ply.so"
         OLD_RPATH "::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io_ply.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_io" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/io/ply" TYPE FILE FILES
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/ply/byte_order.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/ply/io_operators.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/ply/ply.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/ply/ply_parser.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_io" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib/libpcl_io.so.1.10.0"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib/libpcl_io.so.1.10"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib::"
           NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_io" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io.so"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib/libpcl_io.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io.so"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_io.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_io" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/io/pcl_io-1.10.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_io" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/io" TYPE FILE FILES
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/boost.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/eigen.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/debayer.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/fotonic_grabber.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/file_io.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/auto_io.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/low_level_io.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/lzf.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/lzf_image_io.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/io.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/grabber.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/file_grabber.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/pcd_grabber.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/pcd_io.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/vtk_io.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/ply_io.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/tar.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/obj_io.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/ascii_io.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/ifs_io.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/image_grabber.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/hdl_grabber.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/vlp_grabber.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/robot_eye_grabber.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/point_cloud_image_extractors.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/io_exception.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/pxc_grabber.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_io" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/compression" TYPE FILE FILES
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/compression/octree_pointcloud_compression.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/compression/color_coding.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/compression/compression_profiles.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/compression/entropy_range_coder.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/compression/point_coding.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_io" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/io/impl" TYPE FILE FILES
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/impl/ascii_io.hpp"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/impl/pcd_io.hpp"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/impl/auto_io.hpp"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/impl/lzf_image_io.hpp"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/impl/synchronized_queue.hpp"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/io/impl/point_cloud_image_extractors.hpp"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/compression/impl/entropy_range_coder.hpp"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/io/include/pcl/compression/impl/octree_pointcloud_compression.hpp"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/io/tools/cmake_install.cmake")

endif()

