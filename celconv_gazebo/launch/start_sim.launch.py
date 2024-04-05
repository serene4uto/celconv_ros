from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 
                'launch', 
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'worlds', 'empty.world']),
            'verbose': 'true',
            'paused': 'true',
            'gui': 'true',
            'physics': 'dart',
        }.items()
    )

    celconv_spawn_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('celconv_gazebo'), 
                'launch', 
                'spawn_celconv.launch.py'
            ])
        ),
        launch_arguments={
            'namespace': 'celconv',
            'use_sim_time': 'True',
            'num_rows': '5',
            'num_cols': '5',
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(celconv_spawn_node)

    return ld
