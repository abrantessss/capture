#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Get the PX4 location 
    PX4_DIR = os.environ.get('PX4_DIR')
    
    # Get the gazebo package directory
    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    world = os.path.join(get_package_share_directory('capture_gazebo'), 'worlds', 'seixal_track.world')

    return LaunchDescription([
        
        # Add the PX4 gazebo assets to the gazebo plugin and model paths
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', PX4_DIR + '/build/px4_sitl_default/build_gazebo-classic'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', PX4_DIR + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models' + ':' + get_package_share_directory('pegasus_gazebo') + '/models' + ':' + get_package_share_directory('capture_gazebo') + '/models'),

        # Add the launch arguments to select the simulation world and whether to launch the gazebo gui
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=world),

        # Launch the gazebo server with a specific world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzserver.launch.py']),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'verbose': 'true'}.items()),
        
        # Launch the gazebo window to view the simulation, if 'gui' option is set to True
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzclient.launch.py']),
            condition=LaunchConfigurationEquals('gui', 'true')
        )
       
])
