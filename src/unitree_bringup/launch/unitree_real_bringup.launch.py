from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_pkg_share = FindPackageShare("unitree_robot_description")
    bringup_pkg_share = FindPackageShare("unitree_bringup")
    moveit_pkg_share = FindPackageShare("unitree_robot_moveit_config")
    start_gui = LaunchConfiguration("start_gui")
    start_operator = LaunchConfiguration("start_operator")
    start_rviz = LaunchConfiguration("start_rviz")
    serial_port = LaunchConfiguration("serial_port")
    serial_baud_rate = LaunchConfiguration("serial_baud_rate")
    motor_type = LaunchConfiguration("motor_type")
    startup_retry_count = LaunchConfiguration("startup_retry_count")
    default_kp = LaunchConfiguration("default_kp")
    default_kd = LaunchConfiguration("default_kd")
    joint1_motor_id = LaunchConfiguration("joint1_motor_id")
    joint2_motor_id = LaunchConfiguration("joint2_motor_id")
    joint3_motor_id = LaunchConfiguration("joint3_motor_id")
    velocity_scaling = LaunchConfiguration("velocity_scaling")
    acceleration_scaling = LaunchConfiguration("acceleration_scaling")

    urdf_file = PathJoinSubstitution(
        [description_pkg_share, "urdf", "hardware_driver", "unitree_arm.urdf.xacro"]
    )
    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                urdf_file,
                " serial_port:=",
                serial_port,
                " serial_baud_rate:=",
                serial_baud_rate,
                " motor_type:=",
                motor_type,
                " startup_retry_count:=",
                startup_retry_count,
                " default_kp:=",
                default_kp,
                " default_kd:=",
                default_kd,
                " joint1_motor_id:=",
                joint1_motor_id,
                " joint2_motor_id:=",
                joint2_motor_id,
                " joint3_motor_id:=",
                joint3_motor_id,
            ]
        )
    }

    robot_controllers = PathJoinSubstitution(
        [bringup_pkg_share, "config", "unitree_real_controller.yaml"]
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

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_pkg_share, "launch", "moveit_rviz.launch.py"])
        ),
        condition=IfCondition(start_rviz),
    )

    unitree_moveit_operator = Node(
        package="my_controller",
        executable="unitree_moveit_operator",
        output="screen",
        parameters=[
            {
                "velocity_scaling": velocity_scaling,
                "acceleration_scaling": acceleration_scaling,
            }
        ],
        condition=IfCondition(start_operator),
    )

    my_controller_gui = Node(
        package="my_controller",
        executable="my_controller_gui",
        output="screen",
        condition=IfCondition(start_gui),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_gui",
                default_value="false",
                description="Start the my_controller GUI on real hardware.",
            ),
            DeclareLaunchArgument(
                "start_operator",
                default_value="true",
                description="Start the MoveIt operator node on real hardware.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz with the MoveIt visualization config.",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial device used by UnitreeHardwareInterface.",
            ),
            DeclareLaunchArgument(
                "serial_baud_rate",
                default_value="4000000",
                description="Serial baud rate passed to the Unitree SDK SerialPort constructor.",
            ),
            DeclareLaunchArgument(
                "motor_type",
                default_value="GO_M8010_6",
                description="Unitree SDK motor type string. Supported values: GO_M8010_6, A1, B1.",
            ),
            DeclareLaunchArgument(
                "startup_retry_count",
                default_value="3",
                description="Activation startup read retries per motor before failing hardware activation.",
            ),
            DeclareLaunchArgument(
                "default_kp",
                default_value="0.5",
                description="Default Unitree motor position stiffness when controller does not command kp.",
            ),
            DeclareLaunchArgument(
                "default_kd",
                default_value="0.02",
                description="Default Unitree motor damping when controller does not command kd.",
            ),
            DeclareLaunchArgument(
                "joint1_motor_id",
                default_value="0",
                description="Unitree motor id mapped to joint1.",
            ),
            DeclareLaunchArgument(
                "joint2_motor_id",
                default_value="1",
                description="Unitree motor id mapped to joint2.",
            ),
            DeclareLaunchArgument(
                "joint3_motor_id",
                default_value="2",
                description="Unitree motor id mapped to joint3.",
            ),
            DeclareLaunchArgument(
                "velocity_scaling",
                default_value="0.8",
                description="MoveIt velocity scaling for GUI/operator requests, clamped to 0.01..1.0.",
            ),
            DeclareLaunchArgument(
                "acceleration_scaling",
                default_value="0.8",
                description="MoveIt acceleration scaling for GUI/operator requests, clamped to 0.01..1.0.",
            ),
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            move_group,
            moveit_rviz,
            unitree_moveit_operator,
            my_controller_gui,
        ]
    )
