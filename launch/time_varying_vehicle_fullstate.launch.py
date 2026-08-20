from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheel_loader_controller',
            executable='simple_vehicle_controller_fullstate_node',
            name='simple_vehicle_controller_fullstate',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'update_rate': 20.0,

                'initial_velocity': 2.0,
                'initial_steering': 0.0,

                'velocity_step': 1.0,
                'steering_step': 0.1,
                'update_interval_sec': 3.0,

                'max_velocity': 8.0,
                'min_velocity': 0.0,
                'max_steering': 0.5,
                'min_steering': -0.5,
            }]
        )
    ])