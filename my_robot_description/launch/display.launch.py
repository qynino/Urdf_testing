from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    my_robot_description_dir = get_package_share_directory('my_robot_description')

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            my_robot_description_dir,
            "urdf",
            "my_cobot.urdf.xacro"
            ),
        description="Path to robot urdf file"
    )

    robot_desciption = ParameterValue(
        Command(["xacro"," ", LaunchConfiguration("model")])
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desciption}]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d", 
            os.path.join(
                my_robot_description_dir,
                "rviz",
                "urdf_config.rviz"
                )
            ]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])