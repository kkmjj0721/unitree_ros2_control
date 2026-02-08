cmake_minimum_required(VERSION 3.8)
project(unitree_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# 安装 launch 文件夹 (包含 unitree_mock.launch.xml 和 新建的 unitree_real.launch.py)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 config 文件夹 (包含控制器参数)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
