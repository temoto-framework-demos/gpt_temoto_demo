cmake_minimum_required(VERSION 2.8.3)
project(vaultbot_temoto_config)

# use c++11 standard
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  temoto_action_assistant
  temoto_process_manager
  temoto_robot_manager
  temoto_component_manager
  tf
  tf2
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/robot_state
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(joy_to_twsiststamped src/joy_to_twsiststamped.cpp)
target_link_libraries(joy_to_twsiststamped ${catkin_LIBRARIES})
