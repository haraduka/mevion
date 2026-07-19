from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    gui = LaunchConfiguration("gui")
    robot = LaunchConfiguration("robot")

    xacro_file = PathJoinSubstitution([
        FindPackageShare("mevion"),
        "models",
        [LaunchConfiguration("robot"), TextSubstitution(text=".urdf.xacro")]])

    robot_description = Command(["xacro ", xacro_file])

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("robot", default_value="quad_mevion"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}]
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(gui)
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                PathJoinSubstitution([
                    FindPackageShare("mevion"),
                    "config",
                    "urdf.rviz"
                ])
            ]
        ),
    ])
