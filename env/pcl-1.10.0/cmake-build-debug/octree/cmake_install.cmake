# Install script for directory: /home/today/thirtyparty/pcl-1.10.0/octree

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_octree" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_octree.so.1.10.0"
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_octree.so.1.10"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so.1.10"
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_octree" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so"
         RPATH "/usr/local/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_octree.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so"
         OLD_RPATH "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib:"
         NEW_RPATH "/usr/local/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_octree.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_octree" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/octree/pcl_octree-1.10.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_octree" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/octree" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/boost.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree_base.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree_container.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree_impl.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree_nodes.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree_key.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree_pointcloud_density.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree_pointcloud_occupancy.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree_pointcloud_singlepoint.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree_pointcloud_pointvector.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree_pointcloud_changedetector.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree_pointcloud_voxelcentroid.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree_pointcloud.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree_iterator.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree_search.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree2buf_base.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree_pointcloud_adjacency.h"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/octree_pointcloud_adjacency_container.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_octree" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/octree/impl" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/impl/octree_base.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/impl/octree_pointcloud.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/impl/octree2buf_base.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/impl/octree_iterator.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/impl/octree_search.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/impl/octree_pointcloud_voxelcentroid.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/octree/include/pcl/octree/impl/octree_pointcloud_adjacency.hpp"
    )
endif()

