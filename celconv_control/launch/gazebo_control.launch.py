from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Declare the prefix argument
    declare_prefix = DeclareLaunchArgument(
        'prefix', default_value='celconv', description='Prefix of the celconv'
    )

    # Use the prefix in controller names
    prefix = LaunchConfiguration('prefix')

    # Define the command to load and activate the joint_state_broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', '--set-state', 'active',
            Command(['echo ', prefix, '_joint_state_broadcaster'])
        ],
        output='screen',
        shell=True  # Ensure that the command is executed in a shell environment
    )

    # Define the command to load and activate the velocity_controller
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', '--set-state', 'active',
            Command(['echo ', prefix, '_velocity_controller'])
        ],
        output='screen',
        shell=True  # Ensure that the command is executed in a shell environment
    )

    # Initialize the LaunchDescription
    ld = LaunchDescription()

    # Add declared arguments and actions to the LaunchDescription
    ld.add_action(declare_prefix)
    ld.add_action(load_joint_state_broadcaster)

    # Use OnProcessExit to chain the loading of controllers sequentially
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        )
    )

    # Note: Removed the direct addition of load_joint_trajectory_controller to avoid double execution

    return ld
