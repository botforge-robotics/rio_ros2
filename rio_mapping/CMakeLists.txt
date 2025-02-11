cmake_minimum_required(VERSION 3.8)
project(rio_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav2_map_server REQUIRED)

install(
  DIRECTORY launch maps params rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
