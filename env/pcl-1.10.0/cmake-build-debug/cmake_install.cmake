# Install script for directory: /home/today/thirtyparty/pcl-1.10.0

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl" TYPE FILE FILES "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/include/pcl/pcl_config.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pclconfig" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl-1.10/Modules" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/cmake/Modules/FindClangFormat.cmake"
    "/home/today/thirtyparty/pcl-1.10.0/cmake/Modules/FindDSSDK.cmake"
    "/home/today/thirtyparty/pcl-1.10.0/cmake/Modules/FindEigen.cmake"
    "/home/today/thirtyparty/pcl-1.10.0/cmake/Modules/FindEnsenso.cmake"
    "/home/today/thirtyparty/pcl-1.10.0/cmake/Modules/FindOpenNI.cmake"
    "/home/today/thirtyparty/pcl-1.10.0/cmake/Modules/FindOpenNI2.cmake"
    "/home/today/thirtyparty/pcl-1.10.0/cmake/Modules/FindPcap.cmake"
    "/home/today/thirtyparty/pcl-1.10.0/cmake/Modules/FindQhull.cmake"
    "/home/today/thirtyparty/pcl-1.10.0/cmake/Modules/FindRSSDK.cmake"
    "/home/today/thirtyparty/pcl-1.10.0/cmake/Modules/FindRSSDK2.cmake"
    "/home/today/thirtyparty/pcl-1.10.0/cmake/Modules/FindSphinx.cmake"
    "/home/today/thirtyparty/pcl-1.10.0/cmake/Modules/FinddavidSDK.cmake"
    "/home/today/thirtyparty/pcl-1.10.0/cmake/Modules/Findlibusb-1.0.cmake"
    "/home/today/thirtyparty/pcl-1.10.0/cmake/Modules/UseCompilerCache.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pclconfig" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl-1.10" TYPE FILE FILES
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/PCLConfig.cmake"
    "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/PCLConfigVersion.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/common/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/kdtree/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/octree/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/search/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/sample_consensus/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/filters/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/2d/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/geometry/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/io/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/features/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/ml/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/segmentation/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/visualization/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/surface/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/registration/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/keypoints/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/tracking/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/recognition/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/stereo/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/apps/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/cuda/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/outofcore/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/examples/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/gpu/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/people/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/simulation/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/test/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/tools/cmake_install.cmake")
  include("/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/doc/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
