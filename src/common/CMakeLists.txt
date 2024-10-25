cmake_minimum_required(VERSION 3.0.2)
project(common)

file(GLOB CPP_SOURCES ./src/*.cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  message_runtime
  mavros
  nlink_parser
)

set(Eigen3_DIR "/usr/include/eigen3")
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIR})

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Dronestate.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime mavros nlink_parser
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

## 创建可执行文件
add_executable(${PROJECT_NAME} ${CPP_SOURCES})

## 链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
