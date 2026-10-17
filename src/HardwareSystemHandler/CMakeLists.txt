cmake_minimum_required(VERSION 3.10)
project(hardware_system_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================
# Dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_ipc_common REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# =====================
# Shared library (CORE LOGIC)
# =====================
add_library(${PROJECT_NAME} SHARED
  src/fsm.cpp
  src/hsh_node.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../tinyfsm/include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_ipc_common
  diagnostic_msgs
  std_msgs
)

# =====================
# Node wrapper & Test Suite
# =====================
add_executable(hsh_node
  src/hsh_node.cpp
)

target_link_libraries(hsh_node
  ${PROJECT_NAME}
)

ament_target_dependencies(hsh_node
  rclcpp
)

add_executable(test_system_behavior
  src/test_system_behavior.cpp
)

target_link_libraries(test_system_behavior
  ${PROJECT_NAME}
)

ament_target_dependencies(test_system_behavior
  rclcpp
  hardware_ipc_common
)

add_executable(test_homing_behavior
  src/test_homing_behavior.cpp
)

target_link_libraries(test_homing_behavior
  ${PROJECT_NAME}
)

ament_target_dependencies(test_homing_behavior
  rclcpp
  hardware_ipc_common
)

add_executable(shm_viewer
  src/shm_viewer.cpp
)

target_link_libraries(shm_viewer
  ${PROJECT_NAME}
)

ament_target_dependencies(shm_viewer
  rclcpp
  hardware_ipc_common
)

# =====================
# Install
# =====================
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  TARGETS hsh_node test_system_behavior test_homing_behavior shm_viewer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# =====================
# Export
# =====================
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  hardware_ipc_common
  diagnostic_msgs
  std_msgs
)

ament_package()
