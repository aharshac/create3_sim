# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 with diffdrive controller in Gazebo and optionally also in RViz.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, GroupAction, LogInfo
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


def generate_launch_description():
    pkg_create3_control = get_package_share_directory('irobot_create_control')

    namespace = LaunchConfiguration('namespace')

    control_params_file = PathJoinSubstitution(
        [pkg_create3_control, 'config', 'control.yaml'])
    
    # configured_params = ParameterFile(
    #     RewrittenYaml(
    #         source_file=control_params_file,
    #         # root_key='diffdrive_controller',
    #         param_rewrites={
    #             "left_wheel_names": ["robot1/left_wheel_joint"],
    #             "right_wheel_names": ("robot1/right_wheel_joint")
    #         },
    #         convert_types=False,
    #     ),
    #     allow_substs=True,
    # )
    ns_controller_config = LaunchConfiguration('controller_cfg')

    diffdrive_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,  # Namespace is not pushed when used in EventHandler
        parameters=[LaunchConfiguration('controller_cfg')],
        arguments=['diffdrive_controller', '-c', 'controller_manager'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
        output='screen',
    )

    # Ensure diffdrive_controller_node starts after joint_state_broadcaster_spawner
    diffdrive_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                     LogInfo(msg=('Control: ', ns_controller_config)),
                     diffdrive_controller_node],
        )
    )

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
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diffdrive_controller_callback)
    # ld.add_action(tf_namespaced_odom_publisher)
    # ld.add_action(tf_namespaced_base_link_publisher)

    return ld
