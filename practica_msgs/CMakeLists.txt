cmake_minimum_required(VERSION 3.5)
project(practica_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetString.srv"
  "srv/GetPoseStamped.srv")

ament_export_dependencies(rosidl_default_runtime)

ament_package()
