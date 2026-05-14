cmake_minimum_required(VERSION 3.8)
project(unitree_actuator_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 1. 自动检测系统架构 (x86_64 或 arm64) 并选择对应的库
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(SDK_LIB_NAME "libUnitreeMotorSDK_Arm64.so")
else()
  set(SDK_LIB_NAME "libUnitreeMotorSDK_Linux64.so")
endif()


# 2. 安装库文件到 lib 目录
# 保留 SDK 原始文件名，因为库自身的 SONAME/运行时依赖名仍是
# libUnitreeMotorSDK_*.so；只改名安装会导致 ros2_control 插件 dlopen 失败。
install(FILES lib/${SDK_LIB_NAME}
  DESTINATION lib
)

install(FILES lib/${SDK_LIB_NAME}
  DESTINATION lib
  RENAME libunitree_actuator_sdk.so  # 重命名为标准名称，方便链接
)

add_executable(test_unitree_motor
  test/test_unitree_motor.cpp
)
target_include_directories(test_unitree_motor
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/unitree_actuator_sdk
)
target_link_directories(test_unitree_motor
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
)
target_link_libraries(test_unitree_motor
  ${SDK_LIB_NAME}
)

install(TARGETS test_unitree_motor
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 3. 安装头文件
install(DIRECTORY include/ 
  DESTINATION include
)

# 4. 导出依赖
ament_export_include_directories(include include/unitree_actuator_sdk)

# 直接导出库的名字（链接时会自动去 lib 目录下找 libunitree_actuator_sdk.so）
ament_export_libraries(unitree_actuator_sdk) 

ament_package()
