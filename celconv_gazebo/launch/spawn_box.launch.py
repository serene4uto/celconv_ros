from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    declare_box_name = DeclareLaunchArgument(name='box_name', default_value='example_box', description='Name of the box to spawn')
    declare_x = DeclareLaunchArgument(name='x', default_value='0', description='X position of the box')
    declare_y = DeclareLaunchArgument(name='y', default_value='0', description='Y position of the box')
    declare_z = DeclareLaunchArgument(name='z', default_value='0.2', description='Z position of the box')

    box_name = LaunchConfiguration('box_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')

    urdf_box_process = ExecuteProcess(
        cmd=[
            'xacro ' + 
            get_package_share_directory('celconv_gazebo') + '/urdf/box.urdf.xacro' +
            ' > ' + 
            get_package_share_directory('celconv_gazebo') + '/urdf/box.urdf'
        ],
        output='screen',
        shell=True
    )

    spawn_box_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', box_name, 
                   '-file', get_package_share_directory('celconv_gazebo') + '/urdf/box.urdf',
                   '-x', x, '-y', y, '-z', z, '-timeout', '30']
    )

    ld = LaunchDescription()

    ld.add_action(declare_box_name)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_z)
    ld.add_action(urdf_box_process)  # Ensure this action is added to launch description

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=urdf_box_process,
                on_exit=[spawn_box_node],
            )
        )
    )

    return ld
