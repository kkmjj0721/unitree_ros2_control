cmake_minimum_required(VERSION 3.8)
project(my_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/my_controller.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_srvs
  trajectory_msgs
)
pluginlib_export_plugin_description_file(controller_interface my_controller_plugins.xml)

add_executable(unitree_moveit_operator
  src/moveit_operator.cpp
)
target_compile_features(unitree_moveit_operator PUBLIC cxx_std_17)
ament_target_dependencies(unitree_moveit_operator
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  std_msgs
  std_srvs
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS unitree_moveit_operator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES my_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/my_controller_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME my_controller_gui
)


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_srvs
  trajectory_msgs
)

ament_package()
