# Install script for directory: /home/today/thirtyparty/pcl-1.10.0/common

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_common.so.1.10.0"
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_common.so.1.10"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "::::::::::::::"
           NEW_RPATH "/usr/local/lib")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so"
         RPATH "/usr/local/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_common.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so"
         OLD_RPATH "::::::::::::::"
         NEW_RPATH "/usr/local/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_common.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/common/pcl_common-1.10.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/correspondence.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/exceptions.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/pcl_base.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/pcl_exports.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/pcl_macros.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/point_cloud.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/point_traits.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/point_types_conversion.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/point_representation.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/point_types.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/for_each_type.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/pcl_tests.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/cloud_iterator.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/TextureMesh.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/sse.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/PCLPointField.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/PCLPointCloud2.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/PCLImage.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/PCLHeader.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/ModelCoefficients.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/PolygonMesh.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/Vertices.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/PointIndices.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/register_point_struct.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/conversions.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/make_shared.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/common" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/boost.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/angles.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/bivariate_polynomial.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/centroid.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/concatenate.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/common.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/common_headers.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/distances.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/eigen.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/copy_point.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/io.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/file_io.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/intersections.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/norms.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/piecewise_linear_function.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/polynomial_calculations.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/poses_from_matches.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/time.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/time_trigger.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/transforms.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/transformation_from_correspondences.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/vector_average.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/pca.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/point_tests.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/synchronizer.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/utils.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/geometry.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/gaussian.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/point_operators.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/spring.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/intensity.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/random.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/generate.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/projection_matrix.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/colors.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/feature_histogram.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/common/fft" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/fft/_kiss_fft_guts.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/fft/kiss_fft.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/fft/kiss_fftr.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/common/impl" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/angles.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/bivariate_polynomial.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/centroid.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/common.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/eigen.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/intersections.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/copy_point.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/io.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/file_io.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/norms.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/piecewise_linear_function.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/polynomial_calculations.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/pca.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/transforms.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/transformation_from_correspondences.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/vector_average.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/gaussian.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/spring.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/intensity.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/random.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/generate.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/projection_matrix.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/common/impl/accumulators.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/impl" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/impl/pcl_base.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/impl/instantiate.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/impl/point_types.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/impl/cloud_iterator.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/console" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/console/parse.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/console/print.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/console/time.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/range_image" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/range_image/bearing_angle_image.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/range_image/range_image.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/range_image/range_image_planar.h"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/range_image/range_image_spherical.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_common" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/range_image/impl" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/range_image/impl/range_image.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/range_image/impl/range_image_planar.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/common/include/pcl/range_image/impl/range_image_spherical.hpp"
    )
endif()

