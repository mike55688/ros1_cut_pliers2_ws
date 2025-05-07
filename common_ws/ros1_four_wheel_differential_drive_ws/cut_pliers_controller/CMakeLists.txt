cmake_minimum_required(VERSION 3.0.2)
project(cut_pliers_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  serial
  tf
  custom_msgs
)



# 產生訊息
generate_messages(
  DEPENDENCIES
  std_msgs
)

# catkin 包含的訊息與服務
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime serial tf custom_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 加入可執行檔案與編譯設置
add_executable(cut_pliers_controller src/cut_pliers_controller.cpp)
target_link_libraries(cut_pliers_controller ${catkin_LIBRARIES})
add_dependencies(cut_pliers_controller ${catkin_EXPORTED_TARGETS} custom_msgs_generate_messages_cpp)

add_executable(keyboard src/keyboard.cpp)
target_link_libraries(keyboard ${catkin_LIBRARIES})
add_dependencies(keyboard ${catkin_EXPORTED_TARGETS} custom_msgs_generate_messages_cpp)

add_executable(test_node src/test.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})
add_dependencies(test_node ${catkin_EXPORTED_TARGETS} custom_msgs_generate_messages_cpp)

add_executable(keyboard_publisher src/keyboard.cpp)
target_link_libraries(keyboard_publisher ${catkin_LIBRARIES})
add_dependencies(keyboard_publisher ${catkin_EXPORTED_TARGETS} custom_msgs_generate_messages_cpp)

install(TARGETS cut_pliers_controller keyboard test_node keyboard_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
