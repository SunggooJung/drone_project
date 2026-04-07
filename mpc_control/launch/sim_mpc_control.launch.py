from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation mode'
    )

    px4_bridge_node = Node(
        package='drone_mpc_control',
        executable='px4_bridge_node',
        name='px4_bridge',
        output='screen'
    )

    mpc_controller_node = Node(
        package='drone_mpc_control',
        executable='mpc_controller_node',
        name='mpc_controller',
        output='screen',
        parameters=[{
            'control_rate': 20.0,
            'drone_mass': 1.5,
            'mpc_horizon': 20,
            'mpc_dt': 0.05,
        }]
    )

    trajectory_generator_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='drone_mpc_control',
                executable='trajectory_generator_node',
                name='trajectory_generator',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        use_sim_arg,
        px4_bridge_node,
        mpc_controller_node,
        trajectory_generator_node,
    ])
