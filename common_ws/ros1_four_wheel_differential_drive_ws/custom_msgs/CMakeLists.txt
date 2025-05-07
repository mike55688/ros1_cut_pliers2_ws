cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

## 宣告要產生的訊息檔案
add_message_files(
  FILES
  CmdCutPliers.msg
)

## 生成訊息
generate_messages(
  DEPENDENCIES
  std_msgs
)

## 宣告 catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
