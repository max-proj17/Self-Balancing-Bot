
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('balance_bot')

    xacro_file          = os.path.join(pkg_path, 'urdf',         'balance_bot.urdf.xacro')
    gazebo_world_file   = os.path.join(pkg_path, 'gazebo',        'plane.world')
    controller_cfg_file = os.path.join(pkg_path, 'gazebo', 'config', 'controller_config.yaml')

    # ── Process xacro ──────────────────────────────────────────────────────
    import re

    doc = xacro.process_file(xacro_file, mappings={'config_file': controller_cfg_file})
    robot_desc = doc.toxml()


    if robot_desc.startswith('<?xml'):
        robot_desc = robot_desc[robot_desc.index('?>') + 2:]

    robot_desc = re.sub(r'<!--.*?-->', '', robot_desc, flags=re.DOTALL)
    robot_desc = robot_desc.replace('\n', ' ').replace('\r', ' ')
    robot_desc = re.sub(r'\s+', ' ', robot_desc).strip()
    robot_desc = robot_desc.encode('ascii', errors='replace').decode('ascii')

    SPAWN_Z = '0.040'

    # ── Nodes ────────────────────────────────────────────────────────────────

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so',
            gazebo_world_file,
        ],
        output='screen',
        shell=True,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen',
    )


    balance_bot_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='balance_bot',
                executable='balance_bot',
                name='balance_controller',
                parameters=[{'use_sim_time': True}],
                output='screen',
                # Restart automatically if it crashes (e.g. on sim reset)
                respawn=True,
                respawn_delay=1.0,
            )
        ],
    )


    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_entity',
                arguments=[
                    '-entity', 'balance_bot',
                    '-topic',  'robot_description',
                    '-z', SPAWN_Z,
                ],
                output='screen',
            )
        ],
    )


    def make_spawner(controller_name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                controller_name,
                '--controller-manager-timeout', '30',
            ],
            output='screen',
        )

    spawn_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity.actions[0],   # the inner Node
            on_exit=[
                make_spawner('joint_state_broadcaster'),
                make_spawner('left_wheel_controller'),
                make_spawner('right_wheel_controller'),
            ],
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        balance_bot_node,      # t=4.0s — starts waiting for IMU messages
        spawn_entity,          # t=5.0s — entity appears in simulation
        spawn_controllers,     # fires immediately after spawn_entity exits
    ])