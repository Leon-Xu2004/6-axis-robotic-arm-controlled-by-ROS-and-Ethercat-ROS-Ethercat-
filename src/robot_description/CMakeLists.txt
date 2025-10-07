cmake_minimum_required(VERSION 3.0.2)
project(robot_description)

find_package(catkin REQUIRED)

catkin_package()

# 安装 URDF 和 mesh 资源到共享目录
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

catkin_install_python(PROGRAMS
  src/eepath_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)