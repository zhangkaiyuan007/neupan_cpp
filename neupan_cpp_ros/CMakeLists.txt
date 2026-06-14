cmake_minimum_required(VERSION 3.16)
project(neupan_cpp_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(libneupan REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(neupan_node src/neupan_node.cpp)
target_link_libraries(neupan_node neupan::neupan)
ament_target_dependencies(neupan_node
  rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs tf2 tf2_ros)

# Minimal reference global planner (A* on /map -> /initial_path) for demos.
add_executable(astar_global_node src/astar_global_node.cpp)
ament_target_dependencies(astar_global_node
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS neupan_node astar_global_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Vendor the DUNE model from libneupan into our own share, so the launch file
# only depends on this (ament) package's share directory. libneupan is a
# plain-CMake package and is therefore not discoverable via the ament index.
install(DIRECTORY ${libneupan_MODELS_DIR}/ DESTINATION share/${PROJECT_NAME}/models)

ament_package()
