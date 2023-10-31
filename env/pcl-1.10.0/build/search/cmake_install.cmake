# Install script for directory: /home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/search

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

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_search" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_search.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_search.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib/libpcl_search.so.1.10.0"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib/libpcl_search.so.1.10"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_search.so.1.10.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_search.so.1.10"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
           NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_search" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_search.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_search.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_search.so"
         RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib/libpcl_search.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_search.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_search.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_search.so"
         OLD_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib::"
         NEW_RPATH "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0-install/lib:/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/flann-1.9.2-install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpcl_search.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_search" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/build/search/pcl_search-1.10.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_search" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/search" TYPE FILE FILES
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/search/include/pcl/search/search.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/search/include/pcl/search/kdtree.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/search/include/pcl/search/brute_force.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/search/include/pcl/search/organized.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/search/include/pcl/search/octree.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/search/include/pcl/search/flann_search.h"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/search/include/pcl/search/pcl_search.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "pcl_search" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/pcl-1.10/pcl/search/impl" TYPE FILE FILES
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/search/include/pcl/search/impl/search.hpp"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/search/include/pcl/search/impl/kdtree.hpp"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/search/include/pcl/search/impl/flann_search.hpp"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/search/include/pcl/search/impl/brute_force.hpp"
    "/home/sy/SEU_WS/seu_lidarloc_new/seu_lidarloc/env/pcl-1.10.0/search/include/pcl/search/impl/organized.hpp"
    )
endif()

