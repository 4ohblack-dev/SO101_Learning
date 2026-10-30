cmake_minimum_required(VERSION 3.8)
project(so100_hardware)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(FTSERVO_ROOT "$ENV{HOME}/libraries/FTServo_Linux" CACHE PATH
  "Path to the FTServo_Linux checkout")
if(NOT EXISTS "${FTSERVO_ROOT}/src/SMS_STS.h")
  message(FATAL_ERROR "FTServo_Linux not found. Set -DFTSERVO_ROOT=/path/to/FTServo_Linux")
endif()

add_library(feetech_sdk STATIC
  ${FTSERVO_ROOT}/src/SCS.cpp
  ${FTSERVO_ROOT}/src/SCSerial.cpp
  ${FTSERVO_ROOT}/src/SMS_STS.cpp
)
target_include_directories(feetech_sdk PUBLIC ${FTSERVO_ROOT}/src)

add_executable(sts3215_node src/sts3215_node.cpp)
target_compile_features(sts3215_node PRIVATE cxx_std_17)
target_link_libraries(sts3215_node feetech_sdk)
ament_target_dependencies(sts3215_node rclcpp sensor_msgs)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(sts3215_node PRIVATE -Wall -Wextra -Wpedantic)
endif()
install(TARGETS sts3215_node DESTINATION lib/${PROJECT_NAME})
ament_package()
