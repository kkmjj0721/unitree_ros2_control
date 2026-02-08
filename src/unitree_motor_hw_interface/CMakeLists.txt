cmake_minimum_required(VERSION 3.8)
project(unitree_motor_hw_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(unitree_actuator_sdk REQUIRED)


include_directories(include)


# 1. 编译库文件
add_library(${PROJECT_NAME} SHARED
  src/unitree_hardware_interface.cpp
)

# 2、链接依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  unitree_actuator_sdk
)

# 3、导出插件描述文件
pluginlib_export_plugin_description_file(
hardware_interface 
my_unitree_robot.xml
)

# 4、安装库和文件
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES my_unitree_robot.xml
  DESTINATION share/${PROJECT_NAME}
)

# 5、导出依赖
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

ament_package()
