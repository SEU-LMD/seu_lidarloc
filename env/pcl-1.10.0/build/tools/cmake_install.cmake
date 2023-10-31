# Install script for directory: /home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/tools

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_sac_segmentation_plane" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_sac_segmentation_plane")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_sac_segmentation_plane"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_sac_segmentation_plane")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_sac_segmentation_plane" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_sac_segmentation_plane")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_sac_segmentation_plane"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_sac_segmentation_plane")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_plane_projection" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_plane_projection")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_plane_projection"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_plane_projection")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_plane_projection" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_plane_projection")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_plane_projection"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_plane_projection")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_normal_estimation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_normal_estimation")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_normal_estimation"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_normal_estimation")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_normal_estimation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_normal_estimation")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_normal_estimation"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_normal_estimation")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_uniform_sampling" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_uniform_sampling")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_uniform_sampling"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_uniform_sampling")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_uniform_sampling" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_uniform_sampling")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_uniform_sampling"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_uniform_sampling")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_boundary_estimation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_boundary_estimation")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_boundary_estimation"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_boundary_estimation")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_boundary_estimation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_boundary_estimation")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_boundary_estimation"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_boundary_estimation")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_cluster_extraction" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_cluster_extraction")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_cluster_extraction"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_cluster_extraction")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_cluster_extraction" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_cluster_extraction")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_cluster_extraction"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_cluster_extraction")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_fpfh_estimation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_fpfh_estimation")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_fpfh_estimation"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_fpfh_estimation")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_fpfh_estimation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_fpfh_estimation")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_fpfh_estimation"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_fpfh_estimation")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd2ply" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd2ply")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd2ply"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_pcd2ply")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd2ply" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd2ply")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd2ply"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd2ply")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ply2pcd" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ply2pcd")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ply2pcd"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_ply2pcd")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ply2pcd" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ply2pcd")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ply2pcd"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ply2pcd")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_xyz2pcd" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_xyz2pcd")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_xyz2pcd"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_xyz2pcd")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_xyz2pcd" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_xyz2pcd")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_xyz2pcd"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_xyz2pcd")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pclzf2pcd" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pclzf2pcd")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pclzf2pcd"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_pclzf2pcd")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pclzf2pcd" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pclzf2pcd")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pclzf2pcd"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pclzf2pcd")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd2vtk" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd2vtk")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd2vtk"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_pcd2vtk")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd2vtk" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd2vtk")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd2vtk"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd2vtk")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_vfh_estimation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_vfh_estimation")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_vfh_estimation"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_vfh_estimation")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_vfh_estimation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_vfh_estimation")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_vfh_estimation"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_vfh_estimation")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_spin_estimation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_spin_estimation")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_spin_estimation"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_spin_estimation")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_spin_estimation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_spin_estimation")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_spin_estimation"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_spin_estimation")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_voxel_grid" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_voxel_grid")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_voxel_grid"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_voxel_grid")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_voxel_grid" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_voxel_grid")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_voxel_grid"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_voxel_grid")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_passthrough_filter" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_passthrough_filter")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_passthrough_filter"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_passthrough_filter")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_passthrough_filter" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_passthrough_filter")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_passthrough_filter"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_passthrough_filter")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_radius_filter" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_radius_filter")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_radius_filter"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_radius_filter")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_radius_filter" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_radius_filter")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_radius_filter"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_radius_filter")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_extract_feature" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_extract_feature")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_extract_feature"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_extract_feature")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_extract_feature" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_extract_feature")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_extract_feature"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_extract_feature")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_compute_cloud_error" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_compute_cloud_error")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_compute_cloud_error"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_compute_cloud_error")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_compute_cloud_error" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_compute_cloud_error")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_compute_cloud_error"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_compute_cloud_error")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_train_unary_classifier" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_train_unary_classifier")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_train_unary_classifier"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_train_unary_classifier")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_train_unary_classifier" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_train_unary_classifier")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_train_unary_classifier"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_train_unary_classifier")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_unary_classifier_segment" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_unary_classifier_segment")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_unary_classifier_segment"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_unary_classifier_segment")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_unary_classifier_segment" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_unary_classifier_segment")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_unary_classifier_segment"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_unary_classifier_segment")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_crf_segmentation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_crf_segmentation")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_crf_segmentation"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_crf_segmentation")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_crf_segmentation" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_crf_segmentation")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_crf_segmentation"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_crf_segmentation")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_add_gaussian_noise" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_add_gaussian_noise")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_add_gaussian_noise"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_add_gaussian_noise")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_add_gaussian_noise" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_add_gaussian_noise")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_add_gaussian_noise"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_add_gaussian_noise")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_outlier_removal" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_outlier_removal")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_outlier_removal"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_outlier_removal")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_outlier_removal" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_outlier_removal")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_outlier_removal"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_outlier_removal")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_mls_smoothing" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_mls_smoothing")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_mls_smoothing"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_mls_smoothing")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_mls_smoothing" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_mls_smoothing")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_mls_smoothing"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_mls_smoothing")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_marching_cubes_reconstruction" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_marching_cubes_reconstruction")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_marching_cubes_reconstruction"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_marching_cubes_reconstruction")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_marching_cubes_reconstruction" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_marching_cubes_reconstruction")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_marching_cubes_reconstruction"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_marching_cubes_reconstruction")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_gp3_surface" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_gp3_surface")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_gp3_surface"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_gp3_surface")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_gp3_surface" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_gp3_surface")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_gp3_surface"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_gp3_surface")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_icp" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_icp")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_icp"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_icp")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_icp" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_icp")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_icp"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_icp")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_icp2d" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_icp2d")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_icp2d"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_icp2d")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_icp2d" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_icp2d")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_icp2d"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_icp2d")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_elch" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_elch")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_elch"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_elch")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_elch" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_elch")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_elch"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_elch")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_lum" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_lum")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_lum"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_lum")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_lum" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_lum")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_lum"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_lum")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ndt2d" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ndt2d")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ndt2d"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_ndt2d")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ndt2d" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ndt2d")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ndt2d"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ndt2d")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ndt3d" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ndt3d")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ndt3d"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_ndt3d")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ndt3d" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ndt3d")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ndt3d"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_ndt3d")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd_change_viewpoint" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd_change_viewpoint")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd_change_viewpoint"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_pcd_change_viewpoint")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd_change_viewpoint" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd_change_viewpoint")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd_change_viewpoint"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_pcd_change_viewpoint")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_concatenate_points_pcd" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_concatenate_points_pcd")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_concatenate_points_pcd"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_concatenate_points_pcd")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_concatenate_points_pcd" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_concatenate_points_pcd")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_concatenate_points_pcd"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_concatenate_points_pcd")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_poisson_reconstruction" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_poisson_reconstruction")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_poisson_reconstruction"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_poisson_reconstruction")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_poisson_reconstruction" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_poisson_reconstruction")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_poisson_reconstruction"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_poisson_reconstruction")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_train_linemod_template" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_train_linemod_template")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_train_linemod_template"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_train_linemod_template")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_train_linemod_template" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_train_linemod_template")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_train_linemod_template"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_train_linemod_template")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_match_linemod_template" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_match_linemod_template")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_match_linemod_template"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_match_linemod_template")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_match_linemod_template" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_match_linemod_template")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_match_linemod_template"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_match_linemod_template")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_linemod_detection" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_linemod_detection")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_linemod_detection"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_linemod_detection")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_linemod_detection" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_linemod_detection")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_linemod_detection"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_linemod_detection")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_fast_bilateral_filter" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_fast_bilateral_filter")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_fast_bilateral_filter"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_fast_bilateral_filter")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_fast_bilateral_filter" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_fast_bilateral_filter")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_fast_bilateral_filter"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_fast_bilateral_filter")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_demean_cloud" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_demean_cloud")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_demean_cloud"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_demean_cloud")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_demean_cloud" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_demean_cloud")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_demean_cloud"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_demean_cloud")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_compute_hausdorff" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_compute_hausdorff")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_compute_hausdorff"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_compute_hausdorff")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_compute_hausdorff" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_compute_hausdorff")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_compute_hausdorff"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_compute_hausdorff")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_morph" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_morph")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_morph"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_morph")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_morph" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_morph")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_morph"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_morph")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_progressive_morphological_filter" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_progressive_morphological_filter")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_progressive_morphological_filter"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_progressive_morphological_filter")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_progressive_morphological_filter" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_progressive_morphological_filter")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_progressive_morphological_filter"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_progressive_morphological_filter")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_generate" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_generate")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_generate"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_generate")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_generate" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_generate")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_generate"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_generate")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_local_max" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_local_max")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_local_max"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_local_max")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_local_max" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_local_max")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_local_max"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_local_max")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_grid_min" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_grid_min")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_grid_min"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_grid_min")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_grid_min" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_grid_min")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_grid_min"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_grid_min")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_transform_point_cloud" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_transform_point_cloud")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_transform_point_cloud"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_transform_point_cloud")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_transform_point_cloud" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_transform_point_cloud")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_transform_point_cloud"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_transform_point_cloud")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_tools" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_transform_from_viewpoint" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_transform_from_viewpoint")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_transform_from_viewpoint"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/bin/pcl_transform_from_viewpoint")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_transform_from_viewpoint" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_transform_from_viewpoint")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_transform_from_viewpoint"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/pcl_transform_from_viewpoint")
    endif()
  endif()
endif()

