cmake_minimum_required(VERSION 3.8)
project(bumperbot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(trajectory_drawer src/trajectory_drawer.cpp)
ament_target_dependencies(trajectory_drawer rclcpp geometry_msgs nav_msgs)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS
  trajectory_drawer
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/trajectory_drawer.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
