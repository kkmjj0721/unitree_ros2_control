# unitree_ros2_control
基于宇树电机的ros2_control

使用方法：

- 编译：（不要进入src目录）

  ```
  colcon build
  source install/setup.bash
  ```

- 检查模型：

  ```
  ros2 launch unitree_robot_description mock_arm.launch.py
  ```

- 运行虚拟组件 ros2_control：

  ```
  ros2 launch unitree_bringup unitree_mock.launch.py
  ```

- 测试电机：（注意把电机 id 改为0）

  ```
  sudo chmod 777 dev/ttyUSB*
  ros2 run unitree_actuator_sdk test_unitree_motor
  ```
