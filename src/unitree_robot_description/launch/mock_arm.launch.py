import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. 获取包路径
    package_name = 'unitree_robot_description'
    pkg_share = get_package_share_directory(package_name)

    # 2. 指定 xacro 文件路径
    # 根据您的文件结构，mock 文件位于 urdf/mock_components 下
    xacro_file = os.path.join(pkg_share, 'urdf', 'mock_components', 'mock_arm.urdf.xacro')

    # 3. 使用 xacro 命令生成 robot_description
    # 注意：如果在 xacro 中引用其他文件报错，请检查文件路径引用
    robot_description = Command(['xacro ', xacro_file])

    # 4. 配置节点
    # Robot State Publisher: 发布机器人模型描述和 TF
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher GUI: 提供关节控制滑动条窗口
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2: 可视化界面
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # 如果您后续创建了保存的 rviz 配置，可以在这里加载：
        # arguments=['-d', os.path.join(pkg_share, 'config', 'your_config.rviz')]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])