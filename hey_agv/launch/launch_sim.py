from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('hey_agv').find('hey_agv')
    
    # Get paths to config files
    world_path = os.path.join(pkg_share, 'urdf', 'test.world')
    xacro_file = os.path.join(pkg_share, 'urdf', 'dose_car.urdf.xacro')
    bridge_config = '/home/meditab/agv_ws/src/hey_agv/config/ros_gz_bridge.yaml'
    
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Use Command to process xacro
    robot_description_content = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    
    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],  # -r flag to run immediately
        output='screen'
    )

    # Launch Robot State Publisher with delay
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # Delay execution of robot_state_publisher
    delayed_robot_state_publisher = TimerAction(
        period=2.0,  # 2 second delay
        actions=[robot_state_publisher]
    )

    joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    output='screen',
    parameters=[{
        'robot_description': robot_description_content,  # Add this line
        'use_sim_time': use_sim_time
    }]
)

    # Delay execution of joint_state_publisher
    delayed_joint_state_publisher = TimerAction(
        period=5.0,  # 2 second delay
        actions=[joint_state_publisher]
    )

    # ROS-GZ bridge for bidirectional communication
    # Using ExecuteProcess instead of Node for more direct control
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     parameters=[{
    #         'config_file': os.path.join(get_package_share_directory('hey_agv'), 'config', 'ros_gz_bridge.yaml'),
    #         'qos_overrides./tf_static_publisher.durability': 'transient_local'
    #     }],
    #     output='screen',
    # )

    # Increase delay for bridge to ensure everything else is fully initialized
    # delayed_bridge = TimerAction(
    #     period=10.0,  # Increased to 10 second delay
    #     actions=[bridge]
    # )

    # Spawn URDF in Gazebo using ign service
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'agv',
            '-topic', '/robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'  # Slightly above ground to prevent initial collision
        ],
        output='screen'
    )

    # Delay execution of spawn_entity
    delayed_spawn = TimerAction(
        period=1.0,  # 1 second delay after Gazebo starts
        actions=[spawn_entity]
    )

    # Option to run bridge separately
    declare_launch_bridge = DeclareLaunchArgument(
        'launch_bridge',
        default_value='true',
        description='Launch the ROS-GZ bridge (set to false if running separately)'
    )
    
    # Return the launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_launch_bridge,
        gazebo,
        delayed_spawn,
        delayed_robot_state_publisher,
        delayed_joint_state_publisher,
        
    ])