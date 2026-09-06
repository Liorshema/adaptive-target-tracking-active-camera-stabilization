from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # --------------------------------------------------
    # Package paths
    # --------------------------------------------------

    simulation_share = Path(
        get_package_share_directory('rover_simulation')
    )

    description_share = Path(
        get_package_share_directory('my_robot_pkg')
    )

    ros_gz_sim_share = Path(
        get_package_share_directory('ros_gz_sim')
    )

    # --------------------------------------------------
    # Files
    # --------------------------------------------------

    world_file = (
        simulation_share
        / 'worlds'
        / 'greenhouse_flat.sdf'
    )

    xacro_file = (
        description_share
        / 'urdf'
        / 'simple_rover.urdf.xacro'
    )

    config_file = (
        description_share
        / 'config'
        / 'robot'
        / 'rover.yaml'
    )

    # --------------------------------------------------
    # Xacro -> robot_description
    # --------------------------------------------------

    robot_description = ParameterValue(
        Command([
            'xacro ',
            str(xacro_file),
            ' config_file:=',
            str(config_file),
        ]),
        value_type=str,
    )

    # --------------------------------------------------
    # Gazebo world
    # --------------------------------------------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                ros_gz_sim_share
                / 'launch'
                / 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items(),
    )

    # --------------------------------------------------
    # Publish robot_description
    # --------------------------------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
            }
        ],
        output='screen',
    )

    # --------------------------------------------------
    # Spawn rover in Gazebo
    # --------------------------------------------------

    spawn_rover = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'greenhouse_flat',
            '-topic', 'robot_description',
            '-name', 'simple_rover',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
        ],
        output='screen',
    )

    # --------------------------------------------------
    # ROS <-> Gazebo bridge
    # --------------------------------------------------

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

            '/model/simple_rover/odometry'
            '@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],        
        output='screen',
    )

    # --------------------------------------------------
    # Nominal kinematic model
    # --------------------------------------------------
    
    nominal_node = Node(
        package='rover_simulation',
        executable='nominal_kinematics_node',
        parameters=[
            {
                'use_sim_time': True,
            }
        ],
        output='screen',
    )    

    # --------------------------------------------------
    # State comparator
    # --------------------------------------------------

    state_comparator = Node(
        package='rover_simulation',
        executable='state_comparator',
        parameters=[
            {
                'use_sim_time': True,
            }
        ],
        output='screen',
    )
    
    # --------------------------------------------------
    # Launch
    # --------------------------------------------------


    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_rover,
        bridge,
        nominal_node,
        state_comparator,
    ])    