cmake_minimum_required(VERSION 3.8)
project(rio_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rio_description REQUIRED)

install(
  DIRECTORY launch worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
