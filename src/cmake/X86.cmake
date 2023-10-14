find_package(catkin REQUIRED COMPONENTS
        tf
        roscpp
        rospy
        std_msgs
#        pcl_conversion
        sensor_msgs
        geometry_msgs
        nav_msgs
        message_generation
        visualization_msgs
        rosbag
        )
set(Communication_LIBS ${catkin_LIBRARIES})
set(Communication_DIR ${catkin_INCLUDE_DIRS})
