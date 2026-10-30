cmake_minimum_required(VERSION 3.8)
project(so100_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joy_teleop src/joy_teleop.cpp)
target_compile_features(joy_teleop PUBLIC cxx_std_17)
target_include_directories(joy_teleop PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(joy_teleop rclcpp sensor_msgs)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(joy_teleop PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS joy_teleop DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
