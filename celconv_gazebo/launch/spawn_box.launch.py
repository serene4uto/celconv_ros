from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_box_process = ExecuteProcess(
        cmd=[
            'xacro ' + 
            get_package_share_directory('celconv_gazebo') + '/urdf/box.urdf.xacro' +
            ' > ' + 
            get_package_share_directory('celconv_gazebo') + '/urdf/example_box.urdf'
        ],
        output='screen',
        shell=True
    )

    spawn_box_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'example_box', 
                   '-file', get_package_share_directory('celconv_gazebo') + '/urdf/example_box.urdf',
                   '-x', '0.173205', '-y', '-0.1', '-z', '0.2', '-timeout', '30']
    )

    ld = LaunchDescription()

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
