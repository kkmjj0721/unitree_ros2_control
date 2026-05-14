# unitree_ros2_control
基于宇树电机的 ros2_control+moveit2 的三自由度机械臂

# Process:
  运行主入口后，会先读取当前电机的角度，将该机械臂的关节角度赋值，并且移动过去，然后我们可以通过 gui 来进行复位，或者直接发送目标位姿，然后 moveit2 进行规划，自定义 controller 接收 FollowJointTrajectory，然后插值更新 joint position command，然后 hardware_interface 转成电机命令下发；



## Build and source the workspace:
  ```
  colcon build
  source install/setup.bash
  ```


## Usage:
- 测试电机：（注意修改文件内的电机 id，默认为 0）

  ```
  sudo chmod 777 dev/ttyUSB*
  ros2 run unitree_actuator_sdk test_unitree_motor /dev/ttyUSB0 0
  ```

- 修改电机 id：

  ```
  cd UNITREE_ROS2_CONTROL/motor_tools/根据自己主机版本选择对应文件夹
  sudo ./swboot /dev/ttyUSB0
  sudo ./changeid /dev/ttyUSB0 当前id 目标id
  sudo ./swmotor /dev/ttyUSB0
  ```
  
- 检查模型：

  ```
  ros2 launch unitree_robot_description mock_arm.launch.py
  ```

- 运行虚拟组件 ros2_control：

  ```
  ros2 launch unitree_bringup unitree_mock.launch.py
  ```

- 运行 demo:
  ```
  sudo chmod 777 /dev/ttyUSB*
  ros2 launch unitree_bringup unitree_real_bringup.launch.py \
    serial_port:=/dev/ttyUSB0 \
    start_rviz:=true \
    start_gui:=true \
    start_operator:=true
  ```

 ## PS：
 - 宇树电机 sdk 以及工具箱：
  


