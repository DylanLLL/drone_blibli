#!/usr/bin/env python3

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get the warehouse simulation package directory
    pkg_warehouse_simulation = get_package_share_directory('warehouse_simulation')
    
    # Define the world file path
    world_file = PathJoinSubstitution([
        pkg_warehouse_simulation,
        'worlds',
        'warehouse.world'
    ])
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to world file to load'
    )
    
    # PX4 SITL launch arguments
    px4_dir = DeclareLaunchArgument(
        'px4_dir',
        default_value='/home/gdnuser/PX4-Autopilot',
        description='PX4 Autopilot directory'
    )
    
    # Start Gazebo with the warehouse world
    gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', LaunchConfiguration('world')],
        output='screen'
    )
    
    # PX4 SITL process
    px4_cmd = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500'],
        cwd=LaunchConfiguration('px4_dir'),
        output='screen'
    )
    
    # Micro XRCE-DDS Agent
    micro_ros_agent_cmd = ExecuteProcess(
        # cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        cmd=['sudo', '/usr/local/bin/MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )
    
    return LaunchDescription([
        world_arg,
        px4_dir,
        gazebo_cmd,
        px4_cmd,
        micro_ros_agent_cmd,
    ])