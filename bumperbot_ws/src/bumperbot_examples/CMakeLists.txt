cmake_minimum_required(VERSION 3.0.2)
project(bumperbot_examples)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2_ros
  tf2
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  GetTransform.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    tf2_ros
    tf2
    geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
    nodes/simple_tf_examples_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(simple_tf_examples src/simple_tf_examples.cpp)
add_dependencies(simple_tf_examples ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_tf_examples ${catkin_LIBRARIES})

add_executable(simple_tf_examples_node nodes/simple_tf_examples_node.cpp)
add_dependencies(simple_tf_examples_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_tf_examples_node ${catkin_LIBRARIES} simple_tf_examples)