cmake_minimum_required(VERSION 2.8.3)
project(art_bringup)

find_package(catkin REQUIRED COMPONENTS
    roslaunch
    kinect2_bridge
    art_calibration
    art_db
    art_brain
    art_touch_driver
    art_arcode_detector
    art_projected_gui
    art_simple_tracker
    topic_tools
    rosbag
    image_transport
    )

catkin_package()

if (CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()
