cmake_minimum_required(VERSION 3.8)
project(bumperbot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_library(odometry_motion_model SHARED
  src/odometry_motion_model.cpp)
ament_target_dependencies(odometry_motion_model rclcpp nav_msgs geometry_msgs tf2)

add_executable(kalman_filter src/kalman_filter.cpp)
ament_target_dependencies(kalman_filter rclcpp nav_msgs sensor_msgs)

add_executable(imu_republisher src/imu_republisher.cpp)
ament_target_dependencies(imu_republisher rclcpp sensor_msgs)

add_executable(odometry_motion_model_node src/odometry_motion_model_node.cpp)
target_link_libraries(odometry_motion_model_node odometry_motion_model)
ament_target_dependencies(odometry_motion_model_node rclcpp)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  odometry_motion_model
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  kalman_filter
  imu_republisher
  odometry_motion_model_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/kalman_filter.py
  ${PROJECT_NAME}/imu_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
