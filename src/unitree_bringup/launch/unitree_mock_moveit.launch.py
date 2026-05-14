from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_pkg_share = FindPackageShare("unitree_robot_description")
    bringup_pkg_share = FindPackageShare("unitree_bringup")
    moveit_pkg_share = FindPackageShare("unitree_robot_moveit_config")

    urdf_file = PathJoinSubstitution(
        [description_pkg_share, "urdf", "mock_components", "mock_arm.urdf.xacro"]
    )
    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", urdf_file])
    }

    robot_controllers = PathJoinSubstitution(
        [bringup_pkg_share, "config", "unitree_mock_moveit_controller.yaml"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, robot_controllers],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_pkg_share, "launch", "move_group.launch.py"])
        )
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            move_group,
        ]
    )
