from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction


def generate_launch_description():
    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='20.0',
        description='MPC control rate in Hz'
    )

    drone_mass_arg = DeclareLaunchArgument(
        'drone_mass',
        default_value='1.5',
        description='Drone mass in kg'
    )

    mpc_horizon_arg = DeclareLaunchArgument(
        'mpc_horizon',
        default_value='20',
        description='MPC prediction horizon'
    )

    mpc_dt_arg = DeclareLaunchArgument(
        'mpc_dt',
        default_value='0.05',
        description='MPC time step in seconds'
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
            'control_rate': LaunchConfiguration('control_rate'),
            'drone_mass': LaunchConfiguration('drone_mass'),
            'mpc_horizon': LaunchConfiguration('mpc_horizon'),
            'mpc_dt': LaunchConfiguration('mpc_dt'),
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
        control_rate_arg,
        drone_mass_arg,
        mpc_horizon_arg,
        mpc_dt_arg,
        px4_bridge_node,
        mpc_controller_node,
        trajectory_generator_node,
    ])
