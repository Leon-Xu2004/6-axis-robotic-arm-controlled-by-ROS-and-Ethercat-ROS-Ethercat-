cmake_minimum_required(VERSION 3.0.2)
project(robot_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/YS_H7Multi_usart.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
  src/YS_H7Multi_command.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
  src/eRob3_Ecat_TCP.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)