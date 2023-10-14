# Install script for directory: /home/today/thirtyparty/pcl-1.10.0/segmentation

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_segmentation" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_segmentation.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_segmentation.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib:/home/today/thirtyparty/flann-1.9.2-install/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_segmentation.so.1.10.0"
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_segmentation.so.1.10"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_segmentation.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_segmentation.so.1.10"
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_segmentation" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_segmentation.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_segmentation.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_segmentation.so"
         RPATH "/usr/local/lib:/home/today/thirtyparty/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_segmentation.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_segmentation.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_segmentation.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_segmentation.so"
         OLD_RPATH "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib:/home/today/thirtyparty/flann-1.9.2-install/lib:"
         NEW_RPATH "/usr/local/lib:/home/today/thirtyparty/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_segmentation.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_segmentation" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/segmentation/pcl_segmentation-1.10.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_segmentation" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/segmentation" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/boost.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/extract_clusters.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/extract_labeled_clusters.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/extract_polygonal_prism_data.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/min_cut_segmentation.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/sac_segmentation.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/seeded_hue_segmentation.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/segment_differences.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/region_growing.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/region_growing_rgb.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/comparator.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/plane_coefficient_comparator.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/euclidean_plane_coefficient_comparator.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/edge_aware_plane_comparator.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/rgb_plane_coefficient_comparator.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/plane_refinement_comparator.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/euclidean_cluster_comparator.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/ground_plane_comparator.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/organized_connected_component_segmentation.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/organized_multi_plane_segmentation.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/random_walker.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/region_3d.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/planar_region.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/planar_polygon_fusion.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/crf_segmentation.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/crf_normal_segmentation.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/unary_classifier.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/conditional_euclidean_clustering.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/supervoxel_clustering.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/grabcut_segmentation.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/progressive_morphological_filter.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/approximate_progressive_morphological_filter.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/lccp_segmentation.h"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/cpc_segmentation.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_segmentation" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/segmentation/impl" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/extract_clusters.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/extract_labeled_clusters.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/extract_polygonal_prism_data.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/min_cut_segmentation.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/sac_segmentation.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/seeded_hue_segmentation.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/segment_differences.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/random_walker.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/region_growing.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/region_growing_rgb.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/organized_connected_component_segmentation.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/organized_multi_plane_segmentation.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/planar_polygon_fusion.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/crf_segmentation.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/unary_classifier.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/crf_normal_segmentation.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/conditional_euclidean_clustering.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/supervoxel_clustering.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/grabcut_segmentation.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/progressive_morphological_filter.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/approximate_progressive_morphological_filter.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/lccp_segmentation.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/segmentation/include/pcl/segmentation/impl/cpc_segmentation.hpp"
    )
endif()

