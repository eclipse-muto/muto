import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    # Use this file's location to get config files
    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(launch_file_dir)
    config_dir = os.path.join(workspace_dir, 'config')
    
    
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'enable_symphony',
            default_value='true',
            description='Enable Symphony MQTT provider',
            choices=['true', 'false']
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='INFO',
            description='Logging level for all nodes',
            choices=['DEBUG', 'INFO', 'WARN', 'ERROR']
        ),

        DeclareLaunchArgument(
            'muto_config_file',
            default_value= os.path.join(config_dir, "muto.yaml"),
            description='Path to global Muto configuration file'
        ),
        DeclareLaunchArgument("muto_namespace", default_value="muto"),
        DeclareLaunchArgument(
            "vehicle_namespace",
            default_value="org.eclipse.muto.sandbox",
            description="Vehicle ID namespace",
        ),
        DeclareLaunchArgument(
            "vehicle_name",  description="Vehicle ID"
        )
    ]
    
    # Configuration parameters
    enable_symphony = LaunchConfiguration('enable_symphony')
    log_level = LaunchConfiguration('log_level')
    muto_config_file = LaunchConfiguration('muto_config_file')
    vehicle_namespace = LaunchConfiguration('vehicle_namespace')
    vehicle_name = LaunchConfiguration('vehicle_name')
    muto_namespace = LaunchConfiguration('muto_namespace')


    # Agent
    node_agent = Node(
        namespace=muto_namespace,
        name="agent",
        package="agent",
        executable="muto_agent",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
        ],
    )

    node_mqtt_gateway = Node(
        namespace=muto_namespace,
        name="gateway",
        package="agent",
        executable="mqtt",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    node_commands = Node(
        namespace=muto_namespace,
        name="commands_plugin",
        package="agent",
        executable="commands",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Core
    node_twin = Node(
        namespace=muto_namespace,
        name="core_twin",
        package="core",
        executable="twin",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Composer
    node_composer = Node(
        namespace=muto_namespace,
        name="muto_composer",
        package="composer",
        executable="muto_composer",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    node_compose_plugin = Node(
        namespace=muto_namespace,
        name="compose_plugin",
        package="composer",
        executable="compose_plugin",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    node_native_plugin = Node(
        namespace=muto_namespace,
        name="native_plugin",
        package="composer",
        executable="native_plugin",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    node_launch_plugin = Node(
        namespace=muto_namespace,
        name="launch_plugin",
        package="composer",
        executable="launch_plugin",
        output="screen",
        parameters=[
            muto_config_file,
            {"namespace": vehicle_namespace},
            {"name": vehicle_name},
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    symphony_provider = Node(
        namespace=muto_namespace,
        package='agent',
        executable='symphony_provider',
        name='muto_symphony_provider',
        output='screen',
        condition=IfCondition(enable_symphony),
        parameters=[
            muto_config_file,
            {"name": vehicle_name},
            {"namespace": vehicle_namespace},
            {"symphony_target_name": vehicle_name},
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Launch Description Object
    ld = LaunchDescription()

    # add all declared arguments
    for arg in declared_arguments:
        ld.add_action(arg)

    # add all nodes
    ld.add_action(node_agent)
    ld.add_action(node_mqtt_gateway)
    ld.add_action(node_commands)
    ld.add_action(node_twin)
    ld.add_action(node_composer)
    ld.add_action(node_compose_plugin)
    ld.add_action(node_native_plugin)
    ld.add_action(node_launch_plugin)
    ld.add_action(symphony_provider)

    return ld
