cmake_minimum_required(VERSION 3.5)
project(taller_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE DebugWithRelInfo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(practica_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)


set(dependencies
  rclcpp
  std_msgs
  practica_msgs
  rclcpp_lifecycle
  geometry_msgs)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)
  endif()

add_executable(proceso_1 src/proceso_1.cpp)
ament_target_dependencies(proceso_1
  rclcpp
  std_msgs
  practica_msgs)

add_executable(proceso_2 src/proceso_2.cpp)
ament_target_dependencies(proceso_2
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  practica_msgs)


add_executable(proceso_3 src/proceso_3.cpp)
ament_target_dependencies(proceso_3
  rclcpp
  practica_msgs)


install(TARGETS
  proceso_1
  proceso_2
  proceso_3
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
