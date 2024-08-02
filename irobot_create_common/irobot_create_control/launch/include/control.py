# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 with diffdrive controller in Gazebo and optionally also in RViz.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, GroupAction, LogInfo, OpaqueFunction
from launch.conditions import LaunchConfigurationNotEquals
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetParametersFromFile
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('controller_cfg', default_value='',
                          description='Controller config'),
]

def get_namespaced_controller_nodes(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    ns_controller_config = LaunchConfiguration('controller_cfg').perform(context)

    diffdrive_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,  # Namespace is not pushed when used in EventHandler
        parameters=[ns_controller_config],
        arguments=['diffdrive_controller', '-c', 'controller_manager'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
        output='screen',
        # ros_arguments=['--log-level', 'debug']
    )

    # Ensure diffdrive_controller_node starts after joint_state_broadcaster_spawner
    diffdrive_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diffdrive_controller_node],
        )
    )

    return [
        LogInfo(msg=('Control node ns: ', LaunchConfiguration('namespace'))),
        LogInfo(msg=('Control controller_config: ', ns_controller_config)),
        joint_state_broadcaster_spawner,
        diffdrive_controller_callback,
        ]

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    ns_controller_config = LaunchConfiguration('controller_cfg')

    ns_controller_nodes = OpaqueFunction(function=get_namespaced_controller_nodes)


    # Static transform from <namespace>/odom to odom
    # See https://github.com/ros-controls/ros2_controllers/pull/533
    tf_namespaced_odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_namespaced_odom_publisher',
        arguments=['0', '0', '0',
                   '0', '0', '0',
                   'odom', [namespace, '/odom']],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen',
        condition=LaunchConfigurationNotEquals('namespace', '')
    )

    # Static transform from <namespace>/base_link to base_link
    tf_namespaced_base_link_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_namespaced_base_link_publisher',
        arguments=['0', '0', '0',
                   '0', '0', '0',
                   [namespace, '/base_link'], 'base_link'],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen',
        condition=LaunchConfigurationNotEquals('namespace', '')
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(LogInfo(msg=('Control: ', ns_controller_config)))
    ld.add_action(ns_controller_nodes)
    # ld.add_action(tf_namespaced_odom_publisher)
    # ld.add_action(tf_namespaced_base_link_publisher)

    return ld
