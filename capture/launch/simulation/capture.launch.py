#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    
    # ----------------------------------------
    # ----------- SIMULATOR LAUNCH -----------
    # ----------------------------------------

    #Launch seixal world in gazebo
    gazebo_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('capture_gazebo'), 'launch/worlds/seixal.launch.py')),
        launch_arguments={
            'gui':'true',
        }.items()
    )

    capture_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('capture_gazebo'), 'launch/vehicles/capture.launch.py')),
        launch_arguments={ #ENU coordinates
            'x': '-0.999935',
            'y': '4.481597',
            'z': '7.5',
            'launch_pegasus': 'false',
            'vehicle_id': '1',
        }.items()
    )

    # ----------------------------------------
    # -------- CONTROL SYSTEM LAUNCH ---------
    # ----------------------------------------

    #Define which file to use for the drone parameters
    drone_params_file_arg = DeclareLaunchArgument(
        'drone_params',
        default_value=os.path.join(get_package_share_directory('capture'), 'config', 'capture.yaml'),
        description='The directory where the drone parameters such as mass, thrust curve, etc. are defined'
    )

    #Call MAVLINK interface package launch file
    mavlink_interface_launch_file = IncludeLaunchDescription(
        #Grab the launch file for the mavlink interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mavlink_interface'), 'launch/mavlink_interface.launch.py')),
        #Define costume launch arguments/parameters used for the mavlink interface
        launch_arguments={
            'vehicle_id': '1',
            'namespace': 'drone',
            'drone_params': LaunchConfiguration('drone_params'),
            'connection': 'udp://:14540',
            'mavlink_forward': "['']"
        }.items(),
    )

    #Call autopilot package launch file
    autopilot_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('autopilot'), 'launch/autopilot.launch.py')),
        #Define costume launch arguments/parameters used
        launch_arguments={
            'vehicle_id': '1',
            'namespace': 'drone',
            'autopilot_yaml': LaunchConfiguration('drone_params'),
        }.items()
    )
    
    control_claw_node = Node(
        package='capture_claw',        
        executable='control_claw_py',   
        namespace='drone1',      
        name='control_claw_py',         
        output='screen'
    )
    
    # ----------------------------------------
    # ---- RETURN THE LAUNCH DESCRIPTION -----
    # ----------------------------------------
    return LaunchDescription([
        #Launch files for simulation
        gazebo_launch_file,
        capture_launch_file,
        #Launch files for the control system
        drone_params_file_arg,
        mavlink_interface_launch_file,
        autopilot_launch_file,
        control_claw_node,
    ])