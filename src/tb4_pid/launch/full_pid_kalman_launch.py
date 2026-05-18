from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ── 1) Gazebo + TurtleBot4 bringup ──────────────────────────────────
    tb4_gz_dir = get_package_share_directory('turtlebot4_gz_bringup')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_gz_dir, 'launch', 'turtlebot4_gz.launch.py')
        ),
        launch_arguments={'world': 'empty_world_hw'}.items(),
    )

    # ── 2) Kalman filter node (wait for Gazebo to be ready) ─────────────
    kalman_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='tb4_pid',
                executable='kalman',
                name='kalman_node',
                output='screen',
            ),
        ],
    )

    # ── 3) Goal pose publisher (wait for Kalman to initialise) ──────────
    goal_pose_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='tb4_pid',
                executable='goal_pose_test',
                name='goal_pose_test_node',
                output='screen',
            ),
        ],
    )

    # ── 4) PID controller (wait for goal pose to be published) ──────────
    pid_node = TimerAction(
        period=17.0,
        actions=[
            Node(
                package='tb4_pid',
                executable='pid',
                name='pid_node',
                output='screen',
            ),
        ],
    )

    # ── 5) Echo kalman_pose for live monitoring ─────────────────────────
    echo_kalman = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'echo', '/kalman_pose'],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        gazebo_launch,
        kalman_node,
        goal_pose_node,
        pid_node,
        echo_kalman,
    ])