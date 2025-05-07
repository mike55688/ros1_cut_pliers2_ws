cmake_minimum_required(VERSION 3.0.2)
project(four_wheel_car_controller)

## 查找 ROS 依赖
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  serial 
  tf 
  nav_msgs
)

## 声明 catkin package
catkin_package()

## 头文件路径
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

## 生成可执行文件（确保与 launch 文件匹配）
add_executable(car_controller src/car_controller.cpp)
target_link_libraries(car_controller ${catkin_LIBRARIES})

add_executable(car_listener src/car_listener.cpp)
target_link_libraries(car_listener ${catkin_LIBRARIES})

add_executable(tttt src/tttt.cpp)
target_link_libraries(tttt ${catkin_LIBRARIES})
