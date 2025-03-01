#!/usr/bin/env python3
import os
import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, LogInfo, IncludeLaunchDescription, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# Get the home and px4 directories and define where temporary files are placed
PX4_DIR = os.environ.get('PX4_DIR')
PX4_RUN_DIR = os.environ.get('HOME') + '/tmp/px4_run_dir'
os.makedirs(PX4_RUN_DIR, exist_ok=True)

def launch_vehicle(context, *args, **kwargs):

     # Define the model to launch
    vehicle_model = 'pegasus_capture'

    vehicle_id = int(LaunchConfiguration('vehicle_id').perform(context))
    port_increment = vehicle_id - 1

    # Get the environment variables
    environment = os.environ
    environment["PX4_SIM_MODEL"] = 'gazebo-classic_pegasus_capture'
    environment["ROS_VERSION"] = '2'

    # Get the PX4-gazebo directory
    px4_gazebo_dir = os.path.join(PX4_DIR, 'Tools/simulation/gazebo-classic/sitl_gazebo-classic')
    pegasus_models_dir = get_package_share_directory('capture_gazebo')
    
    # Get the standard iris drone models inside the PX4 package
    model = os.path.join(pegasus_models_dir, 'models', vehicle_model, vehicle_model + '.sdf')
    
    model_generator_process = ExecuteProcess(
        cmd=[
            os.path.join(px4_gazebo_dir, 'scripts/jinja_gen.py'),
            os.path.join(pegasus_models_dir, 'models/' + vehicle_model + '/' + vehicle_model + '.sdf.jinja'),
            pegasus_models_dir,
            '--mavlink_id=' + str(vehicle_id),
            '--mavlink_udp_port=' + str(14540 + port_increment),
            '--mavlink_tcp_port=' + str(4560 + port_increment),
            '--gst_udp_port=' + str(5600 + port_increment),
            '--video_uri=' + str(5600 + port_increment),
            '--mavlink_cam_udp_port=' + str(14530 + port_increment),
            '--output-file=' + os.path.join(pegasus_models_dir, 'models/'+ vehicle_model + '/' + vehicle_model + str(vehicle_id) + '.sdf'),
            '--generate_ros_models=True'
        ],
        env=environment,
        output='screen',
    )
    
    # Spawn the 3D model in the gazebo world (it requires that a gzserver is already running)
    spawn_3d_model = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'drone' + str(vehicle_id), 
               '-file', os.path.join(pegasus_models_dir, 'models/'+ vehicle_model + '/' + vehicle_model + str(vehicle_id) + '.sdf'),
               '-x', LaunchConfiguration('x').perform(context),
               '-y', LaunchConfiguration('y').perform(context),
               '-z', LaunchConfiguration('z').perform(context),
               '-R', LaunchConfiguration('R').perform(context),
               '-P', LaunchConfiguration('P').perform(context),
               '-Y', LaunchConfiguration('Y').perform(context),
               '-robot_namespace', 'drone' + str(vehicle_id)],
        output='screen'
    )

    # Launch PX4 simulator
    px4_sitl_process = ExecuteProcess(
        cmd=[
            'bash', '-c',
            PX4_DIR + '/build/px4_sitl_default/bin/px4',
            PX4_DIR + '/ROMFS/px4fmu_common/',
            '-s',
            PX4_DIR + '/ROMFS/px4fmu_common/init.d-posix/rcS',
            '-i ' + str(port_increment)
        ],
        cwd=PX4_RUN_DIR,
        output='screen',
        env=environment,
        shell=False
    )
    
    # Launch the pegasus control and navigation code stack
    pegasus_launch = IncludeLaunchDescription(
        # Grab the launch file for the mavlink interface
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('capture'), 'launch/simulation/capture.launch.py')),
        # Define costume launch arguments/parameters used for the mavlink interface
        launch_arguments={
            'id': str(vehicle_id), 
            'namespace': 'drone',
            'connection': 'udp://:' + str(14540 + port_increment)
        }.items(),
        condition=LaunchConfigurationEquals('launch_pegasus', 'true')
    )

    return [model_generator_process,
        
        # After the sdf model generator finishes, then launch the vehicle spawn in gazebo
        RegisterEventHandler(
            OnProcessExit(
                target_action=model_generator_process,
                on_exit=[
                    LogInfo(msg='Vehicle SDF model generated'),
                    spawn_3d_model
                ]
            )
        ),

        # After the sdf model of the vehicle get's spawn on the vehicle, execute the PX4 simulator
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_3d_model,
                on_exit=[
                    LogInfo(msg='Vehicle spawned in gazebo'),
                    px4_sitl_process
                ]
            )
        ),
        
        # Launch the pegasus file that is used to spawn the pegasus control and navigation code stack
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_3d_model,
                on_exit=[
                    LogInfo(msg='Launching the Pegasus code stack!'),
                    pegasus_launch
                ]
            )
        )]

def generate_launch_description():
    """Launch Gazebo with a drone running PX4 communicating over ROS 2."""
    
   
    # Generate the 3D model by replacing in the mavlink configuration parameters, according to the vehicle ID
    return LaunchDescription([
        
        # Define the environment variables so that gazebo can discover PX4 3D models and plugins
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', PX4_DIR + '/build/px4_sitl_default/build_gazebo'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', PX4_DIR + '/Tools/sitl_gazebo/models' + ':' + get_package_share_directory('pegasus_gazebo') + '/models'),
        SetEnvironmentVariable('PX4_SIM_MODEL', 'capture'),

        # Define where to spawn the vehicle (in the inertial frame) 
        # TODO - receive coordinates in ned perform the conversion to ENU and f.l.u here
        # so that the user only needs to work in NED coordinates
        DeclareLaunchArgument('vehicle_id', default_value='1', description='Drone ID in the network'),
        DeclareLaunchArgument('x', default_value='-0.999935', description='X position expressed in ENU'),
        DeclareLaunchArgument('y', default_value='4.481597', description='Y position expressed in ENU'),
        DeclareLaunchArgument('z', default_value='8 ', description='Z position expressed in ENU'),       
        DeclareLaunchArgument('R', default_value='0.0', description='Roll orientation expressed in ENU'),
        DeclareLaunchArgument('P', default_value='0.0', description='Pitch orientation expressed in ENU'),
        DeclareLaunchArgument('Y', default_value='0.0', description='Yaw orientation expressed in ENU'),
        
        # Default to launch the pegasus control stack with the default configurations for the capture vehicle
        DeclareLaunchArgument('launch_pegasus', default_value='true'),
        OpaqueFunction(function=launch_vehicle)
])
