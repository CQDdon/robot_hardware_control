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
find_package(odrive_can_interface REQUIRED)
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
  odrive_can_interface
  diagnostic_msgs
  std_msgs
)

# =====================
# Node wrapper
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

# =====================
# Install
# =====================
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  TARGETS hsh_node
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
  odrive_can_interface
  diagnostic_msgs
  std_msgs
)

ament_package()
