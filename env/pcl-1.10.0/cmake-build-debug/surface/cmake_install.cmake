# Install script for directory: /home/today/thirtyparty/pcl-1.10.0/surface

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_surface" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_surface.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_surface.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib:/home/today/thirtyparty/flann-1.9.2-install/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_surface.so.1.10.0"
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_surface.so.1.10"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_surface.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_surface.so.1.10"
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_surface" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_surface.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_surface.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_surface.so"
         RPATH "/usr/local/lib:/home/today/thirtyparty/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_surface.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_surface.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_surface.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_surface.so"
         OLD_RPATH "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib:/home/today/thirtyparty/flann-1.9.2-install/lib:"
         NEW_RPATH "/usr/local/lib:/home/today/thirtyparty/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_surface.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_surface" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/surface/pcl_surface-1.10.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_surface" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/surface" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/boost.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/eigen.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/ear_clipping.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/gp3.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/grid_projection.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/marching_cubes.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/marching_cubes_hoppe.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/marching_cubes_rbf.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/bilateral_upsampling.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/mls.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/organized_fast_mesh.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/reconstruction.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/processing.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/simplification_remove_unused_vertices.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/surfel_smoothing.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/texture_mapping.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/poisson.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_surface" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/surface/3rdparty/poisson4" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/allocator.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/binary_node.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/bspline_data.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/factor.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/function_data.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/geometry.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/marching_cubes_poisson.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/mat.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/multi_grid_octree_data.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/octree_poisson.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/polynomial.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/ppolynomial.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/sparse_matrix.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/vector.h"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/bspline_data.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/function_data.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/geometry.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/mat.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/multi_grid_octree_data.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/octree_poisson.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/polynomial.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/ppolynomial.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/sparse_matrix.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/vector.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/3rdparty/poisson4/poisson_exceptions.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_surface" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/surface/impl" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/impl/gp3.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/impl/grid_projection.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/impl/marching_cubes.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/impl/marching_cubes_hoppe.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/impl/marching_cubes_rbf.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/impl/bilateral_upsampling.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/impl/mls.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/impl/organized_fast_mesh.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/impl/reconstruction.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/impl/processing.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/impl/surfel_smoothing.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/impl/texture_mapping.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/surface/include/pcl/surface/impl/poisson.hpp"
    )
endif()

