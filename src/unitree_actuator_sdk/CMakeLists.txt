cmake_minimum_required(VERSION 3.8)
project(unitree_actuator_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# =========================================================
# 1. 自动检测系统架构 (x86_64 或 arm64) 并选择对应的库
# =========================================================
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(SDK_LIB_NAME "libUnitreeMotorSDK_Arm64.so")
else()
  set(SDK_LIB_NAME "libUnitreeMotorSDK_Linux64.so")
endif()

# =========================================================
# 2. 创建一个 IMPORTED 库目标 (关键修改)
#    这样 CMake 就知道有一个叫 unitree_sdk 的库，对应具体的 .so 文件
# =========================================================
add_library(unitree_sdk SHARED IMPORTED) # 告诉 CMake：有一个库叫 unitree_sdk，它已经存在了，不是我现编译的
set_target_properties(unitree_sdk PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/${SDK_LIB_NAME}" # 它的具体位置在这里
)

# example
add_executable(test_unitree_motor test/test_unitree_motor.cpp)                # 生成可执行文件
# 包含头文件目录
target_include_directories(test_unitree_motor PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/unitree_actuator_sdk>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_unitree_motor unitree_sdk)                         # 链接 SDK 动态库

# =========================================================
# 3. 安装头文件、库文件、测试文件
# =========================================================
install(DIRECTORY include/ 
  DESTINATION include
)

install(FILES lib/${SDK_LIB_NAME} 
  DESTINATION lib
)

install(TARGETS test_unitree_motor
  DESTINATION lib/${PROJECT_NAME}
)

# =========================================================
# 4. 导出依赖信息给上层包使用
# =========================================================
ament_export_include_directories(include)
ament_export_libraries(unitree_sdk)

ament_package()

