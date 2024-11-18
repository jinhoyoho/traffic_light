cmake_minimum_required(VERSION 3.0.2)
project(traffic_light)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  traffic_srv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  LIBRARIES traffic_light
  CATKIN_DEPENDS cv_bridge roscpp rospy std_msgs sensor_msgs
)

add_executable(traffic_light src/traffic.cpp)
target_link_libraries(traffic_light
  ${catkin_LIBRARIES}
)