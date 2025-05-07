cmake_minimum_required(VERSION 2.8.3)
project(forklift_server)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy  # 添加 rospy
  actionlib 
  actionlib_msgs 
  geometry_msgs 
  nav_msgs  # 添加 nav_msgs
  tf 
  forklift_msg 
  message_generation
)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES TopologyMap.action
  FILES PBVS.action
  FILES PBVSMegapose.action
)

add_message_files(
  FILES
  Detection.msg
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs nav_msgs  # 添加 nav_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs rospy nav_msgs  # 添加 rospy 和 nav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# 安裝 Python 腳本
catkin_install_python(PROGRAMS
  node/pose_visualization.py
  node/PBVS_server_megapose_differential_drive.py
  node/ctrl_server_megapose_differential_drive.py
  node/ctrl_server_megapose.py
  node/ctrl_server.py
  node/ctrl_server_visual.py
  node/PBVS_server_megapose.py
  node/PBVS_server_minicar.py
  node/PBVS_server.py
  node/Topology_map_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安裝 launch 文件（可選）
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)