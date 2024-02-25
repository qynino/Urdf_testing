import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    my_robot_description = get_package_share_directory('my_robot_description')
    
    model_arg = DeclareLaunchArgument(name='model', 
                                      default_value=os.path.join(my_robot_description, 'urdf', 'my_cobot.urdf.xacro'),
                                      description='Absolute path to robot urdf file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': True}]
    )

    load_gazebo= ExecuteProcess(
        cmd=['gazebo', "-s","libgazebo_ros_factory.so"],
        output='screen')

    spawn_robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'my_robot',
                                   '-topic', 'robot_description',
                                  ],
                        output='screen',
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen',
    )

    load_joint_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','effort_controllers'],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        load_gazebo,
        spawn_robot,
        load_joint_state_controller,
        load_joint_effort_controller
        ])