cmake_minimum_required(VERSION 3.0.2)
project(visp_megapose)

## 設定 C++ 編譯標準與警告
add_compile_options(-Wall -Wextra -std=c++11)

## 找到 ROS 1 所需的套件
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  sensor_msgs
  std_msgs
  image_transport
  roscpp
  tf
  visp_bridge
  cv_bridge
  rospy
  message_generation
  forklift_server
)

## 設定自編譯 ViSP 的目錄
# set(VISP_DIR /home/user/visp-ws/visp-build)
# set(CMAKE_PREFIX_PATH ${VISP_DIR} ${CMAKE_PREFIX_PATH})


## 找到 ViSP
set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(VISP REQUIRED)
find_package(OpenCV REQUIRED)

## 設定 include 路徑
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${VISP_DIR}/include
  ${VISP_DIR}/include/visp3
)

## 設定 library 路徑
link_directories(${VISP_DIR}/lib ${VISP_LIBRARY_DIRS})

## 定義 ROS Messages
add_message_files(
  FILES
  Confidence.msg
)

## 定義 ROS Services
add_service_files(
  FILES
  Init.srv
  Render.srv
  Track.srv
)

## 產生 Messages 和 Services
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

## 設定 catkin_package
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  geometry_msgs
  message_filters
  sensor_msgs
  std_msgs
  image_transport
  roscpp
  visp_bridge
  cv_bridge
  forklift_server
  DEPENDS VISP
)

## 安裝 Python 腳本
# catkin_install_python(PROGRAMS
#   scripts/megapose_server_.py
#   scripts/test.py
#   scripts/ros_pose_hz_to_csv.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## 建立 megapose_client 可執行檔
add_executable(megapose_client src/megapose_client.cpp)
add_dependencies(megapose_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(megapose_client
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(depth_image_visualizer src/depth_image_visualizer.cpp)
add_dependencies(depth_image_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(depth_image_visualizer
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## 安裝 Launch 檔案
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## 安裝 Data 資料夾
install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

## 安裝 JSON 變數設定檔
install(FILES 
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/megapose_variables_final.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
