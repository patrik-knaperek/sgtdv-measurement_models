cmake_minimum_required(VERSION 3.0.2)
project(measurement_models)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  sgtdv_msgs
  tf
  eigen_conversions
)

find_package (Eigen3)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../
)

add_executable(${PROJECT_NAME} src/measurement_models.cpp src/measurement_models_node.cpp src/measurement_models_synch.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Eigen3_INCLUDE_DIRS}
)
