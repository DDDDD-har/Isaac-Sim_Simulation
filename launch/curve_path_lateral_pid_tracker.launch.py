from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheel_loader_controller',           
            executable='curve_path_lateral_pid_tracker_node', 
            name='curve_path_lateral_pid_tracker',       
            output='screen',                             
            parameters=[{                                
                'use_sim_time': True,                    # 使用仿真时间
                
                # PID调参
                'kp': 0.22,               # 比例系数 - 减小以防止高速时的蛇形摆动
                'ki': 0.005,              # 积分系数 - 使用最小值以避免超调
                'kd': 0.45,               # 微分系数 - 增大以提高阻尼并补偿相位滞后
                
                'target_velocity': 3.0,   # 目标速度 (rad/s)
                'articulation_rate_limit': 0.06, # 关节响应速率限制，允许更快的关节响应
                
                # 路径定义
                'path_x_start': 0.0,      # 路径 x 轴起点
                'path_x_end': 150.0,      # 路径 x 轴终点
                'path_dx': 0.2,           # 路径点间距
                'path_amplitude': 2.8,    # 路径振幅
                'path_frequency': 0.07,   # 路径频率
                'path_y_offset': 2.7581,  # 路径 y 轴偏移量
                
                'stop_at_end': True,      # 到达终点时停止
                'stop_dist': 1.5,         # 停止距离 (米)
            }]
        )
    ])