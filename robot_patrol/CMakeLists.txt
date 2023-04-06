cmake_minimum_required(VERSION 3.5)
project(robot_patrol)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp  REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(patrol_node src/patrol.cpp)
ament_target_dependencies(patrol_node rclcpp sensor_msgs geometry_msgs custom_interfaces)

add_executable(patrolv2_node src/patrol_v2.cpp)
ament_target_dependencies(patrolv2_node rclcpp sensor_msgs geometry_msgs custom_interfaces)

add_executable(direction_service_node src/direction_service.cpp)
ament_target_dependencies(direction_service_node rclcpp sensor_msgs geometry_msgs custom_interfaces)

add_executable(test_service_node src/test_service.cpp)
ament_target_dependencies(test_service_node rclcpp sensor_msgs geometry_msgs custom_interfaces)


# Install executable
install(TARGETS patrol_node direction_service_node test_service_node patrolv2_node DESTINATION lib/${PROJECT_NAME})

# Install launch file
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install package.xml
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs)



ament_package()
