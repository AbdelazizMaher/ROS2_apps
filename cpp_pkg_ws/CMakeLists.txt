cmake_minimum_required(VERSION 3.5)
project(cpp_pkg_ws)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces)

add_executable(counter_reset src/counter_reset.cpp)
ament_target_dependencies(counter_reset rclcpp example_interfaces)

install(TARGETS
  number_publisher
  number_counter
  counter_reset
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
