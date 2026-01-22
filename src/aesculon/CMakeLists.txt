cmake_minimum_required(VERSION 3.8)
project(aesculon)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Judgment.action"
  DEPENDENCIES action_msgs
)

install(PROGRAMS
  aesculon/core_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aesculon_core
)

install(PROGRAMS
  aesculon/console_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aesculon_console
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
