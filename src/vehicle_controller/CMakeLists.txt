cmake_minimum_required(VERSION 3.8)
project(vehicle_controller)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(vehicle_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/vehicle_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface hardware_interface pluginlib rclcpp realtime_tools vehicle_msgs)
# Controllers are loaded with pluginlib.  Link the selected RMW implementation
# explicitly so its generated message typesupport is already resident when RCL
# creates the subscriptions from this plugin.
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-as-needed)
target_link_libraries(${PROJECT_NAME}
  ${vehicle_msgs_LIBRARIES__rosidl_typesupport_fastrtps_cpp})
pluginlib_export_plugin_description_file(controller_interface vehicle_controller.xml)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES vehicle_controller.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(controller_interface hardware_interface pluginlib rclcpp realtime_tools vehicle_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vehicle_controller_sequence test/test_vehicle_controller_sequence.cpp)
  target_include_directories(test_vehicle_controller_sequence PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_vehicle_controller_sequence
    ${PROJECT_NAME}
    ${vehicle_msgs_LIBRARIES__rosidl_typesupport_fastrtps_cpp})
  ament_target_dependencies(test_vehicle_controller_sequence
    controller_interface hardware_interface rclcpp realtime_tools vehicle_msgs)
endif()

ament_package()
