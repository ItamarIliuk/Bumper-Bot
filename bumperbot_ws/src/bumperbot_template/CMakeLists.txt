cmake_minimum_required(VERSION 3.0.2)
project(bumperbot_template)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib
)

catkin_python_setup()

add_message_files(
  FILES
  BumperbotMsg.msg
)

add_service_files(
  FILES
  BumperbotSrv.srv
)

add_action_files(
  FILES
  Bumperbot.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)


catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
    nodes/bumperbot_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(bumperbot src/bumperbot.cpp)
add_dependencies(bumperbot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bumperbot ${catkin_LIBRARIES})

add_executable(bumperbot_node nodes/bumperbot_node.cpp src/bumperbot.cpp)
add_dependencies(bumperbot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bumperbot_node ${catkin_LIBRARIES})