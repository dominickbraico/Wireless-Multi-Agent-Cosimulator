cmake_minimum_required(VERSION 3.8)
project(radio_relay)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(radio_relay src/radio_relay.cpp)
ament_target_dependencies(radio_relay rclcpp)
install(TARGETS radio_relay DESTINATION lib/${PROJECT_NAME})

ament_package()
