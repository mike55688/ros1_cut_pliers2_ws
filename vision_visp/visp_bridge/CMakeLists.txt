cmake_minimum_required(VERSION 3.0.0)
project(visp_bridge)

## 設定自編譯 ViSP 的目錄
set(VISP_DIR /home/user/visp-ws/visp-build)
set(CMAKE_PREFIX_PATH ${VISP_DIR} ${CMAKE_PREFIX_PATH})

find_package(Boost REQUIRED COMPONENTS filesystem program_options system thread)

find_package(catkin REQUIRED COMPONENTS
  camera_calibration_parsers
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)

# 找到 ViSP
find_package(VISP REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include

  LIBRARIES
    ${PROJECT_NAME}

  CATKIN_DEPENDS
    camera_calibration_parsers
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs

  DEPENDS
    Boost
    VISP
)

include_directories(include ${VISP_DIR}/include)
include_directories(SYSTEM
  ${VISP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 確保 Boost.Filesystem v3 被使用
add_definitions(-DBOOST_FILESYSTEM_VERSION=3)

## 建立 `visp_bridge` 庫
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/3dpose.h
  include/${PROJECT_NAME}/camera.h
  include/${PROJECT_NAME}/image.h
  include/${PROJECT_NAME}/vpQuaternionVector.h
  src/compat/vpQuaternionVector.cpp
  src/conversions/3dpose.cpp
  src/conversions/camera.cpp
  src/conversions/image.cpp
)

target_link_libraries(${PROJECT_NAME} ${VISP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## 建立 `convert_camera_parameters` 可執行檔
add_executable(${PROJECT_NAME}_convert_camera_parameters src/conversions/convert_cam_param_file.cpp)
target_link_libraries(${PROJECT_NAME}_convert_camera_parameters ${VISP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME})

## 安裝可執行檔與庫
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_convert_camera_parameters 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安裝標頭文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
