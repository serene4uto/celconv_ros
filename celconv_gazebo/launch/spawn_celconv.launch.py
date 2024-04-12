from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnIncludeLaunchDescription



def generate_launch_description():
    
    # Declare launch arguments for robot name and initial XYZ position
    declare_namespace = DeclareLaunchArgument('namespace', default_value='', description='Namespace of the celconv')
    declare_x = DeclareLaunchArgument('x', default_value='0', description='Initial x position of the celconv')
    declare_y = DeclareLaunchArgument('y', default_value='0', description='Initial y position of the celconv')
    declare_z = DeclareLaunchArgument('z', default_value='0', description='Initial z position of the celconv')

    declare_num_rows = DeclareLaunchArgument('num_rows', default_value='1', description='Number of rows in the cell grid')
    declare_num_cols = DeclareLaunchArgument('num_cols', default_value='1', description='Number of columns in the cell grid')

    # Use LaunchConfigurations for dynamic parameter retrieval
    celconv_namespace = LaunchConfiguration('namespace')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
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
            'use_sim_time': 'true',
            'num_rows': num_rows,
            'num_cols': num_cols
        }.items()
    )

    spawn_celconv_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace=celconv_namespace,
        arguments=['-entity', celconv_namespace, 
                   '-topic', 'robot_description', 
                   '-x', x, '-y', y, '-z', z, '-timeout', '30']
    )

    celcon_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('celconv_control'), 
                'launch', 
                'gazebo_control.launch.py'
            ])
        ),
        launch_arguments={
            'namespace': celconv_namespace
        }.items()
    )

    gen_control_config_process = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([FindExecutable(name='python3')]),
            PathJoinSubstitution([FindPackageShare('celconv_gazebo'), 'script', 'gazebo_controller_config.py']),
            celconv_namespace,
            num_rows,
            num_cols
        ],
        output='screen'
    )

    # Assemble the Launch Description
    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_z)
    ld.add_action(declare_num_rows)
    ld.add_action(declare_num_cols)
    ld.add_action(gen_control_config_process)

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=gen_control_config_process,
                on_exit=[celconv_description_launch, spawn_celconv_node, celcon_control_launch],
            )
        )
    )

    # ld.add_action(
    #     RegisterEventHandler(
    #         OnIncludeLaunchDescription(
    #             target_action=celconv_description_launch,
    #             on_exit=[, ],
    #         )
    #     )
    # )

    


    return ld
