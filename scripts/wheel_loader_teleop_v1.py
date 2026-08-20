#!/usr/bin/env python3
import sys
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

def get_key():
    import termios, tty, select
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], 0.05)
        return sys.stdin.read(1).lower() if r else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

class WheelLoaderFinalTeleop(Node):
    def __init__(self):
        super().__init__('loader_v1')
        self.names = [
            'FRwheel_revolute', 'FLwheel_revolute', 'boom_cylinder_revolute',
            'boom_pivot_revolute', 'chassis_body_revolute', 'boom_cylinder_prismatic',
            'rocker_bucket_revolute', 'rocker_revolute', 'RLwheel_revolute',
            'RRwheel_revolute', 'rocker_link_revolute', 'rocker_cylinder_revolute',
            'bucket_cylinder_prismatic'
        ]
        self.wheel_vel = 0.0
        self.steer_pos = 0.0
        self.boom_pos = 0.0
        self.bucket_pos = 0.0
        
        self.pub = self.create_publisher(JointState, '/joint_command', 10)
        self.timer = self.create_timer(0.02, self.publish)
        self.running = True
        threading.Thread(target=self.key_loop, daemon=True).start()

        print("\n" + "="*50)
        print("  WheelLoader 控制器 V1 (单消息分流)")
        print("="*50)
        print("  W / S : 前进 / 后退")
        print("  A / D : 左转 / 右转")
        print("  R / F : 动臂 抬升 / 下降")
        print("  T / G : 铲斗 收起 / 翻开")
        print("  Q     : 退出 (程序会立即停止)")
        print("="*50 + "\n")

    def key_loop(self):
        while self.running:
            k = get_key()
            if k == 'w': self.wheel_vel = min(self.wheel_vel + 1.0, 15.0)
            elif k == 's': self.wheel_vel = max(self.wheel_vel - 1.0, -15.0)
            elif k == 'a': self.steer_pos = min(self.steer_pos + 0.05, 0.6)
            elif k == 'd': self.steer_pos = max(self.steer_pos - 0.05, -0.6)
            elif k == 'r': self.boom_pos = min(self.boom_pos + 0.05, 1.2)
            elif k == 'f': self.boom_pos = max(self.boom_pos - 0.05, -0.5)
            elif k == 't': self.bucket_pos = min(self.bucket_pos + 0.08, 1.2)
            elif k == 'g': self.bucket_pos = max(self.bucket_pos - 0.08, -1.2)
            elif k == ' ': self.wheel_vel = 0.0
            elif k == 'q': self.running = False; return
            if k == '': self.wheel_vel *= 0.95
            sys.stdout.write(f"\r速度:{self.wheel_vel:>4.1f} | 转向:{math.degrees(self.steer_pos):>3.0f}° | 动臂:{self.boom_pos:>4.2f} | 铲斗:{self.bucket_pos:>4.2f} ")
            sys.stdout.flush()

    def publish(self):
        if not self.running: return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = [0.0]*13
        msg.velocity = [0.0]*13
        
        # 填充
        for i in [0, 1, 8, 9]: msg.velocity[i] = float(self.wheel_vel)
        msg.position[3] = float(self.boom_pos)
        msg.position[4] = float(self.steer_pos)
        msg.position[6] = float(self.bucket_pos)
        
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = WheelLoaderFinalTeleop()
    try:
        while rclpy.ok() and node.running: rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()