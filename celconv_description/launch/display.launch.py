import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='celconv_description').find('celconv_description')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_view.rviz')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation (Gazebo) clock if true')
    declare_namespace = DeclareLaunchArgument('namespace', default_value='', description='Namespace of the celconv')
    declare_num_rows = DeclareLaunchArgument('num_rows', default_value='1', description='Number of rows in the cell grid')
    declare_num_cols = DeclareLaunchArgument('num_cols', default_value='1', description='Number of columns in the cell grid')

    use_sim_time = LaunchConfiguration('use_sim_time')
    celconv_namespace = LaunchConfiguration('namespace')
    num_rows = LaunchConfiguration('num_rows')
    num_cols = LaunchConfiguration('num_cols')

    celconv_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('celconv_description'), 
                'launch', 
                'description.launch.py'
            ])
        ),
        launch_arguments={
            'namespace': celconv_namespace,
            'use_sim_time': use_sim_time,
            'num_rows': num_rows,
            'num_cols': num_cols
        }.items()
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )



    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(name='gui', default_value='True', 
                        description='Flag to enable joint_state_publisher_gui'))
    ld.add_action(DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                        description='Absolute path to rviz config file'))
    
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_namespace)
    ld.add_action(declare_num_rows)
    ld.add_action(declare_num_cols)
    ld.add_action(celconv_description_launch)

    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)

    return ld