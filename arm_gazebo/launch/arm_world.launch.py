from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
     
def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file", #this will be the name of the argument  
            default_value=PathJoinSubstitution(
                [FindPackageShare("arm_description"), "config", "arm_config.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )
            
    arm_path = os.path.join(get_package_share_directory('arm_gazebo'))

    xacro_arm = os.path.join(arm_path, "urdf", "arm.urdf.xacro")
    
    robot_arm_description = {"robot_description": Command(['xacro ', xacro_arm])}
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_arm_description,
                    {"use_sim_time": True},
            ],
    )
    
    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config_file")],
    )
    
    declared_arguments.append(DeclareLaunchArgument('gz_args', default_value='-r -v 1 empty.sdf',
                              description='Arguments for gz_sim'),)
    
    gazebo_ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'robot_arm',
                   '-allow_renaming', 'true',],
    )
    
    ign = [gazebo_ignition, gz_spawn_entity]
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )  

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],  
    ) 
    
    delay_joint_pos_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[position_controller],
        )
    )
    
    delay_joint_state_broadcaster = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster],
            )
        )
    )
    
    bridge_camera = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args', 
            '-r', '/camera:=/videocamera',
        ],
        output='screen'
    )
    
    nodes_to_start = [
       robot_state_publisher_node,  
       *ign,
       delay_joint_pos_controller, 
       delay_joint_state_broadcaster,
       rviz_node,
       bridge_camera,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start) 
