from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    declare_namespace = DeclareLaunchArgument('namespace', default_value='', description='Namespace of the celconv')
    declare_num_rows = DeclareLaunchArgument('num_rows', default_value='1', description='Number of rows in the cell grid')
    declare_num_cols = DeclareLaunchArgument('num_cols', default_value='1', description='Number of columns in the cell grid')

    name_space = LaunchConfiguration('namespace')
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

    gen_control_joint_config = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([FindExecutable(name='python3')]),
            PathJoinSubstitution([FindPackageShare('celconv_gazebo'), 'script', 'gazebo_controller_config.py']),
            name_space,
            num_rows,
            num_cols
        ],
        output='screen'
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
            'namespace': name_space,
            'use_sim_time': 'True',
            'num_rows': num_rows,
            'num_cols': num_cols,
        }.items()
    )

    celcon_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('celconv_control'), 
                'launch', 
                'gazebo_control.launch.py'
            ])
        ),
        # launch_arguments={
        #     'prefix': name_space
        # }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_num_rows)
    ld.add_action(declare_num_cols)

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=gen_control_joint_config,
                on_exit=[gazebo_launch, celconv_spawn_node, celcon_control_launch],
            )
        )
    )

    ld.add_action(gen_control_joint_config)
    
    return ld
