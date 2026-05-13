# unitree_ros2_control
基于宇树电机的 ros2_control+moveit2 的三自由度机械臂


## Build and source the workspace:
  ```
  colcon build
  source install/setup.bash
  ```


## Usage:
- 测试电机：（注意修改文件内的电机 id，默认为 0）

  ```
  sudo chmod 777 dev/ttyUSB*
  ros2 run unitree_actuator_sdk test_unitree_motor
  ```

- 修改电机 id：

  ```
  cd UNITREE_ROS2_CONTROL/motor_tools/根据自己主机版本选择对应文件夹
  sudo ./swboot /dev/ttyUSB0
  sudo ./changed /dev/ttyUSB0 当前id 目标id
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



