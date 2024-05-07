cmake_minimum_required(VERSION 3.8)
project(merlin2_hospital_patrolling)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  merlin2_hospital_patrolling/merlin2_room_patrol_mission_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_room_patrol_mission_node
)

install(PROGRAMS
  merlin2_hospital_patrolling/merlin2_room_patrol_fsm_action.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_room_patrol_fsm_action
)

install(DIRECTORY
  launch params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()