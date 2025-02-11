cmake_minimum_required(VERSION 3.8)
project(rio_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Auth.action"
  "action/Sms.action"
  "action/TTS.action"
  "msg/Ws2812b.msg"
  "srv/Expression.srv"
  "srv/GetExpression.srv"
  "srv/Camera.srv"
  DEPENDENCIES std_msgs
)

ament_package()
