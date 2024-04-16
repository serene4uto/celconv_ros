from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

import sys
import math

def generate_launch_description():

    # Declare launch arguments
    declare_celconv_namespace = DeclareLaunchArgument('namespace', default_value='', description='Namespace of the celconv')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true')
    declare_pub_joint_states = DeclareLaunchArgument('pub_joint_states', default_value='true', description='Publish joint states')
    declare_num_rows = DeclareLaunchArgument('num_rows', default_value='1', description='Number of rows in the cell grid')
    declare_num_cols = DeclareLaunchArgument('num_cols', default_value='1', description='Number of columns in the cell grid')


    # Use LaunchConfigurations for dynamic parameter retrieval
    use_sim_time = LaunchConfiguration('use_sim_time')
    pub_joint_states = LaunchConfiguration('pub_joint_states')
    celconv_namespace = LaunchConfiguration('namespace')
    num_rows = LaunchConfiguration('num_rows')
    num_cols = LaunchConfiguration('num_cols')

    nd_num_rows = 0
    nd_num_cols = 0
    cam_dist = 1.0
    for arg in sys.argv:
        if 'num_rows' in arg:
            nd_num_rows = int(arg.split(':=')[1])
        if 'num_cols' in arg:
            nd_num_cols = int(arg.split(':=')[1])
    rd = 0.12
    cd = 0.06928 * 1.5
    h = 1.5*nd_num_rows*rd
    w = nd_num_cols*cd
    fov_h = 2 * math.atan((w / 2) / cam_dist)
    fov_v = 2 * math.atan((h / 2) / cam_dist)
    

    # Get the URDF file
    celconv_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('celconv_description'), 'urdf', 'celconv_main.urdf.xacro']
            ),
            ' ',
            'num_rows:=', num_rows,
            ' ',
            'num_cols:=', num_cols,
            ' ',
            'is_sim:=', use_sim_time,
            ' ',
            'cam_fov_h:=', '0.83775804'     #str(fov_h),
            ' ',
            'prefix:=', celconv_namespace,
        ]),
        value_type=str
    )

    # Robot State Publisher Node
    celconv_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=celconv_namespace,
        parameters=[{
            'robot_description': celconv_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=celconv_namespace,
        output='screen',
        condition=IfCondition(pub_joint_states),
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_num_rows)
    ld.add_action(declare_num_cols)
    ld.add_action(declare_celconv_namespace)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_pub_joint_states)


    ld.add_action(celconv_state_publisher_node)
    ld.add_action(joint_state_publisher_node)

    return ld