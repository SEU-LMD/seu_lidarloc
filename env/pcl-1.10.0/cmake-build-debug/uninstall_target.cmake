if(NOT EXISTS "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/install_manifest.txt")
  message(FATAL_ERROR "Cannot find install manifest: \"/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/install_manifest.txt\"")
endif()

file(READ "/home/today/thirtyparty/pcl-1.10.0/cmake-build-debug/install_manifest.txt" files)
string(REGEX REPLACE "\n" ";" files "${files}")
foreach(file ${files})
  message(STATUS "Uninstalling \"$ENV{DESTDIR}${file}\"")
  if(EXISTS "$ENV{DESTDIR}${file}" OR IS_SYMLINK "$ENV{DESTDIR}${file}")
    exec_program("/home/today/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/231.9225.21/bin/cmake/linux/x64/bin/cmake" ARGS "-E remove \"$ENV{DESTDIR}${file}\""
                 OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
    if(NOT "${rm_retval}" STREQUAL 0)
      message(FATAL_ERROR "Problem when removing \"$ENV{DESTDIR}${file}\"")
    endif()
  else()
    message(STATUS "File \"$ENV{DESTDIR}${file}\" does not exist.")
  endif()
endforeach()

# remove pcl directory in include (removes all files in it!)
message(STATUS "Uninstalling \"/usr/local/include/pcl-1.10\"")
if(EXISTS "/usr/local/include/pcl-1.10")
  exec_program("/home/today/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/231.9225.21/bin/cmake/linux/x64/bin/cmake"
               ARGS "-E remove_directory \"/usr/local/include/pcl-1.10\""
               OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
  if(NOT "${rm_retval}" STREQUAL 0)
    message(FATAL_ERROR "Problem when removing \"/usr/local/include/pcl-1.10\"")
  endif()
else()
  message(STATUS "Directory \"/usr/local/include/pcl-1.10\" does not exist.")
endif()

# remove pcl directory in share (removes all files in it!)
# created by CMakeLists.txt for PCLConfig.cmake
if(EXISTS "/usr/local/share/pcl-1.10")
  file(GLOB_RECURSE CMAKE_CONFIG_FOLDER_FILES FOLLOW_SYMLINKS
       LIST_DIRECTORIES false
       "/usr/local/share/pcl-1.10/*")
  list(LENGTH CMAKE_CONFIG_FOLDER_FILES CMAKE_CONFIG_FOLDER_FILES_NUMBER)
  if(CMAKE_CONFIG_FOLDER_FILES_NUMBER EQUAL 0)
    message(STATUS "Uninstalling \"/usr/local/share/pcl-1.10\"")
    exec_program("/home/today/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/231.9225.21/bin/cmake/linux/x64/bin/cmake"
                 ARGS "-E remove_directory \"/usr/local/share/pcl-1.10\""
                 OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
    if(NOT "${rm_retval}" STREQUAL 0)
      message(FATAL_ERROR "Problem when removing \"/usr/local/share/pcl-1.10\"")
    endif()
  endif()
else()
  message(STATUS "Directory \"/usr/local/share/pcl-1.10\" does not exist.")
endif()

# remove pcl directory in share/doc (removes all files in it!)
if(OFF)
  message(STATUS "Uninstalling \"/usr/local/share/doc/pcl-1.10\"")
  if(EXISTS "/usr/local/share/doc/pcl-1.10")
    exec_program("/home/today/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/231.9225.21/bin/cmake/linux/x64/bin/cmake"
                 ARGS "-E remove_directory \"/usr/local/share/doc/pcl-1.10\""
                 OUTPUT_VARIABLE rm_out RETURN_VALUE rm_retval)
    if(NOT "${rm_retval}" STREQUAL 0)
      message(FATAL_ERROR "Problem when removing \"/usr/local/share/doc/pcl-1.10\"")
    endif()
  else()
    message(STATUS "Directory \"/usr/local/share/doc/pcl-1.10\" does not exist.")
  endif()
endif()
