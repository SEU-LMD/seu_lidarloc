set(CPACK_PACKAGE_NAME "PCL")
set(CPACK_PACKAGE_VENDOR "PointClouds.org")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Point Cloud Library (PCL)")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "PCL 1.10.0")
set(CPACK_RESOURCE_FILE_LICENSE "/home/today/fuse/seu_lidarloc/env/pcl-1.10.0/LICENSE.txt")
set(CPACK_RESOURCE_FILE_README "/home/today/fuse/seu_lidarloc/env/pcl-1.10.0/AUTHORS.txt")


set(CPACK_COMPONENT_PCLCONFIG_GROUP "PCL")

set(CPACK_COMPONENT_PCLCONFIG_DISPLAY_NAME "PCLConfig")

set(CPACK_COMPONENT_PCLCONFIG_DESCRIPTION "Helper cmake configuration scripts used by find_package(PCL)")

set(CPACK_COMPONENTS_ALL pclconfig)


if((WIN32 OR UNIX) AND (CPACK_GENERATOR STREQUAL "NSIS"))
    set(CPACK_NSIS_DISPLAY_NAME "PCL-1.10.0")
    set(CPACK_NSIS_MUI_ICON "/home/today/fuse/seu_lidarloc/env/pcl-1.10.0/cmake/images/pcl.ico")
    set(CPACK_NSIS_MUI_UNIICON "/home/today/fuse/seu_lidarloc/env/pcl-1.10.0/cmake/images/pcl.ico")
    set(CPACK_NSIS_HELP_LINK "http://www.pointclouds.org")
    set(CPACK_NSIS_URL_INFO_ABOUT "http://www.pointclouds.org")
    set(CPACK_NSIS_MODIFY_PATH ON)
    set(CPACK_PACKAGE_EXECUTABLES )
    set(CPACK_NSIS_MENU_LINKS
            "share/doc/pcl-1.10/tutorials/html/index.html" "Tutorials"
            "share/doc/pcl-1.10/tutorials/html/sources" "Tutorials sources"
            "share/doc/pcl-1.10/html/pcl-1.10.chm" "Documentation"
            "http://www.pointclouds.org" "PCL Website")
    #set(CPACK_NSIS_MENU_LINKS "share/doc/PCL/user_guide.pdf" "User's guide")
    #set(CPACK_NSIS_MENU_LINKS "share/doc/PCL/developer_guide.pdf" "Developer's guide")
    if(WIN32 AND NOT UNIX)
      # There is a bug in NSI that does not handle full unix paths properly. Make
      # sure there is at least one set of four (4) backlasshes.
      set(CPACK_PACKAGE_ICON "/home/today/fuse/seu_lidarloc/env/pcl-1.10.0/cmake/images\\\\pcl_horz_large_pos.bmp")
    else()
      set(CPACK_PACKAGE_ICON "/home/today/fuse/seu_lidarloc/env/pcl-1.10.0/cmake/images/pcl_horz_large_pos.bmp")
    endif()
endif()

if(UNIX AND ((CPACK_GENERATOR STREQUAL "DEB") OR (CPACK_GENERATOR STREQUAL "RPM")))
    # define stuff for the DEB/RPM packages
    set(CPACK_PACKAGE_CONTACT "pcl-developers@pointclouds.org")
endif()

if(UNIX AND (CPACK_GENERATOR STREQUAL "DEB"))
  set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
endif()

if(APPLE AND (CPACK_GENERATOR STREQUAL "PackageMaker"))
    # define stuff for the PackageMaker packages
    set(CPACK_OSX_PACKAGE_VERSION 10.5)
    set(CPACK_PACKAGE_CONTACT "pcl-developers@pointclouds.org")
    set(CPACK_SET_DESTDIR ON)
    set(CPACK_PACKAGING_INSTALL_PREFIX /usr/local)
endif()
