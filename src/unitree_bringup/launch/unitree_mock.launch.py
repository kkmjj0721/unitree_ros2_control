import os
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 定义包路径查找
    description_pkg_share = FindPackageShare("unitree_robot_description")
    bringup_pkg_share = FindPackageShare("unitree_bringup")

    # 2. 设置机器人描述 (URDF)
    # 引用来源: src/unitree_robot_description/urdf/mock_components/mock_arm.urdf.xacro
    urdf_file = PathJoinSubstitution(
        [description_pkg_share, "urdf", "mock_components", "mock_arm.urdf.xacro"]
    )
    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", urdf_file]
    )
    robot_description = {"robot_description": robot_description_content}

    # 3. 设置控制器配置文件
    # 引用来源: src/unitree_bringup/config/unitree_mock_controller.yaml
    robot_controllers = PathJoinSubstitution(
        [bringup_pkg_share, "config", "unitree_mock_controller.yaml"]
    )

    # 4. 设置 RViz 配置文件路径
    # 注意：这需要 unitree_robot_description 包安装了 rviz 文件夹
    rviz_config_file = PathJoinSubstitution(
        [description_pkg_share, "rviz", "unitree_mock_config.rviz"]
    )

    # 5. 定义节点
    
    # Robot State Publisher（TF）
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ROS2 Control Node
    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # Spawner: Joint State Broadcaster
    node_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawner: Arm Controller
    node_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_joints_controller", "--controller-manager", "/controller_manager"],
    )

    # RViz2 (强制启动，并加载配置文件)
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        # arguments=["-d", rviz_config_file],       # 加载配置文件
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_controller_manager,
        node_joint_state_broadcaster,
        node_arm_controller,
        node_rviz,
    ])