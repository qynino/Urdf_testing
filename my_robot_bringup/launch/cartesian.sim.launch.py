import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration
)

def generate_launch_description():
    my_robot_description = get_package_share_directory('my_robot_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    model_arg = DeclareLaunchArgument(name='model', 
                                      default_value=os.path.join(my_robot_description, 'urdf', 'my_cobot.urdf.xacro'),
                                      description='Absolute path to robot urdf file')

    my_robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("my_robot_description"),
                    "urdf",
                    "my_cobot.urdf.xacro",
                ]
            )
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("my_robot_description"),
            "config",
            "controller_manager.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            # ("motion_control_handle/target_frame", "target_frame"),
            # ("cartesian_motion_controller/target_frame", "target_frame"),
            ("cartesian_compliance_controller/target_frame", "target_frame"),
            # ("cartesian_force_controller/target_wrench", "target_wrench"),
            # ("cartesian_compliance_controller/target_wrench", "target_wrench"),
            # ("cartesian_force_controller/ft_sensor_wrench", "ft_sensor_wrench"),
            # ("cartesian_compliance_controller/ft_sensor_wrench", "wrench"),
        ],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': my_robot_description},
                    {'use_sim_time': True}]
    )

    # load_gazebo= ExecuteProcess(
    #     cmd=['gazebo', "-s","libgazebo_ros_factory.so"],
    #     output='screen')
    
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        )
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    spawn_robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'my_robot',
                                   '-topic', 'robot_description',
                                  ],
                        output='screen',
    )

    def controller_spawner(name, *args):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[name] + [a for a in args],
        )

    # Active controllers
    active_list = [
        "joint_state_broadcaster",
    ]
    active_spawners = [controller_spawner(controller) for controller in active_list]

    # Inactive controllers
    inactive_list = [
        "cartesian_compliance_controller",
        #"cartesian_force_controller",
        #"cartesian_motion_controller",
        #"motion_control_handle",
        #"joint_trajectory_controller",
        #"invalid_cartesian_compliance_controller",
        #"invalid_cartesian_force_controller"
    ]
    state = "--inactive"
    inactive_spawners = [controller_spawner(controller, state) for controller in inactive_list]

    cartesian_motion_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output='screen',
        arguments=["cartesian_motion_controller", "--controller-manager", "/controller_manager"])

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        # load_gazebo,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot,
        control_node,
        cartesian_motion_controller_spawner
        ]
        #+active_spawners
        #+inactive_spawners)
    )