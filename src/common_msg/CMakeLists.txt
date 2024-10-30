cmake_minimum_required(VERSION 3.0.2)
project(common_msg)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

## 添加消息文件
add_message_files(
  FILES
  DroneState.msg
)

## 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## 声明 catkin 包
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

## 包含路径
include_directories(
  ${catkin_INCLUDE_DIRS}
)
