cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_to_2dmap_ros)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  cv_bridge
  sensor_msgs
  roscpp
)

find_package(PCL REQUIRED)
find_package(OpenCV 4.7.0 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(pointcloud_to_2dmap_ros src/pointcloud_to_2dmap_ros.cpp)
target_link_libraries(pointcloud_to_2dmap_ros
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
