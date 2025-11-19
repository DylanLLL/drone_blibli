#!/usr/bin/env python3

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Set to true for verbose output'
    )
    
    # Start Gazebo with the warehouse world
    gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', LaunchConfiguration('world')],
        output='screen'
    )
    
    # Alternative Gazebo command for non-verbose mode
    # gazebo_cmd_quiet = ExecuteProcess(
    #     cmd=['gz', 'sim', '-r', LaunchConfiguration('world')],
    #     condition=launch.conditions.UnlessCondition(
    #         launch.substitutions.LaunchConfiguration('verbose')
    #     ),
    #     output='screen'
    # )
    
    return LaunchDescription([
        world_arg,
        verbose_arg,
        gazebo_cmd,
        # gazebo_cmd_quiet,
    ])