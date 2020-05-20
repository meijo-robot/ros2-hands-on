cmake_minimum_required(VERSION 3.5)
project(turtle_polygon)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(turtle_polygon src/turtle_polygon.cpp)
ament_target_dependencies(turtle_polygon rclcpp rclcpp_action turtlesim  geometry_msgs)

install(TARGETS
  turtle_polygon
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
