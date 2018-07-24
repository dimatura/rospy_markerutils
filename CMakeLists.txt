cmake_minimum_required(VERSION 2.8.3)
project(rospy_markerutils)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    rospy
    rospy_geomutils
    std_msgs
    visualization_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rospy_geomutils
   CATKIN_DEPENDS geometry_msgs rospy rospy_geomutils std_msgs visualization_msgs
#  DEPENDS system_lib
)
