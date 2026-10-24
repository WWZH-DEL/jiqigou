cmake_minimum_required(VERSION 3.0)
project(cmdvel2lcm_py)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_package()

install(PROGRAMS
  scripts/cmdvel_to_lcm.py
  scripts/cmd_vel_msg.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
