from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    declare_num_rows = DeclareLaunchArgument(
        'num_rows', default_value='1', description='Number of rows in the cell grid')
    declare_num_cols = DeclareLaunchArgument(
        'num_cols', default_value='1', description='Number of columns in the cell grid')

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
            'world': PathJoinSubstitution([FindPackageShare('celconv_gazebo'), 'worlds', 'empty.world']),
            'verbose': 'true',
            'paused': 'true',
            'gui': 'true',
            # 'physics': 'ode',
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
            'namespace': '',
            'use_sim_time': 'true',
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
