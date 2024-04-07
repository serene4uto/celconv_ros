from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    declare_num_rows = DeclareLaunchArgument('num_rows', default_value='1', description='Number of rows in the cell grid')
    declare_num_cols = DeclareLaunchArgument('num_cols', default_value='1', description='Number of columns in the cell grid')

    num_rows = LaunchConfiguration('num_rows')
    num_cols = LaunchConfiguration('num_cols')
    
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
            'num_rows': num_rows,
            'num_cols': num_cols,
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_num_rows)
    ld.add_action(declare_num_cols)
    ld.add_action(gazebo_launch)
    ld.add_action(celconv_spawn_node)

    return ld
