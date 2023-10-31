# Install script for directory: /home/today/thirtyparty/pcl-1.10.0/registration

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_registration" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib:/home/today/thirtyparty/flann-1.9.2-install/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_registration.so.1.10.0"
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_registration.so.1.10"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so.1.10"
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_registration" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so"
         RPATH "/usr/local/lib:/home/today/thirtyparty/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib/libpcl_registration.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so"
         OLD_RPATH "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/lib:/home/today/thirtyparty/flann-1.9.2-install/lib:"
         NEW_RPATH "/usr/local/lib:/home/today/thirtyparty/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_registration.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_registration" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/registration/pcl_registration-1.10.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_registration" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/registration" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/eigen.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/boost.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/boost_graph.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/convergence_criteria.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/default_convergence_criteria.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_estimation.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_estimation_normal_shooting.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_estimation_backprojection.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_estimation_organized_projection.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_rejection.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_rejection_distance.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_rejection_median_distance.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_rejection_surface_normal.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_rejection_features.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_rejection_one_to_one.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_rejection_poly.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_rejection_sample_consensus.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_rejection_sample_consensus_2d.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_rejection_trimmed.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_rejection_var_trimmed.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_rejection_organized_boundary.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_sorting.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/correspondence_types.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/ia_ransac.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/icp.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/joint_icp.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/incremental_registration.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/icp_nl.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/lum.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/elch.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/meta_registration.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/ndt.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/ndt_2d.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/ppf_registration.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/pairwise_graph_registration.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/pyramid_feature_matching.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/registration.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/transforms.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/transformation_estimation.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/transformation_estimation_2D.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/transformation_estimation_svd.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/transformation_estimation_svd_scale.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/transformation_estimation_dual_quaternion.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/transformation_estimation_lm.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/transformation_estimation_point_to_plane.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/transformation_estimation_point_to_plane_weighted.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/transformation_estimation_point_to_plane_lls.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/transformation_estimation_symmetric_point_to_plane_lls.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/transformation_validation.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/transformation_validation_euclidean.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/gicp.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/gicp6d.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/bfgs.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/warp_point_rigid.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/warp_point_rigid_6d.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/warp_point_rigid_3d.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/distances.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/exceptions.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/sample_consensus_prerejective.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/ia_fpcs.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/ia_kfpcs.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/matching_candidate.h"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/transformation_estimation_3point.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_registration" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/registration/impl" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/default_convergence_criteria.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/correspondence_estimation.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/correspondence_estimation_normal_shooting.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/correspondence_estimation_backprojection.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/correspondence_estimation_organized_projection.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/correspondence_rejection_distance.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/correspondence_rejection_median_distance.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/correspondence_rejection_surface_normal.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/correspondence_rejection_features.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/correspondence_rejection_one_to_one.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/correspondence_rejection_poly.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/correspondence_rejection_sample_consensus.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/correspondence_rejection_sample_consensus_2d.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/correspondence_rejection_trimmed.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/correspondence_rejection_var_trimmed.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/correspondence_rejection_organized_boundary.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/correspondence_types.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/ia_ransac.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/icp.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/joint_icp.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/incremental_registration.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/icp_nl.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/elch.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/lum.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/meta_registration.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/ndt.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/ndt_2d.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/ppf_registration.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/pyramid_feature_matching.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/registration.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/transformation_estimation_2D.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/transformation_estimation_svd.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/transformation_estimation_svd_scale.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/transformation_estimation_dual_quaternion.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/transformation_estimation_lm.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/transformation_estimation_point_to_plane_lls.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/transformation_estimation_point_to_plane_lls_weighted.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/transformation_estimation_point_to_plane_weighted.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/transformation_estimation_symmetric_point_to_plane_lls.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/transformation_validation_euclidean.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/gicp.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/sample_consensus_prerejective.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/ia_fpcs.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/ia_kfpcs.hpp"
    "/home/today/thirtyparty/pcl-1.10.0/registration/include/pcl/registration/impl/transformation_estimation_3point.hpp"
    )
endif()

