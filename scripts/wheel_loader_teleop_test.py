"""
Simplified Test - Wheels Only
Test if wheels can move without other joints
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys

def get_key():
    import termios, tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1).lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

class WheelsOnlyTest(Node):
    def __init__(self):
        super().__init__('wheels_only_test')
        self.pub = self.create_publisher(JointState, '/joint_command', 10)
        self.timer = self.create_timer(0.02, self.publish_cmd)
        self.vel = 0.0
        print("Wheels Only Test")
        print("W - faster, S - slower, Q - quit")
    
    def publish_cmd(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # ONLY WHEELS - no other joints
        msg.name = [
            'FRwheel_revolute',
            'FLwheel_revolute', 
            'RRwheel_revolute',
            'RLwheel_revolute',
        ]
        msg.position = [0.0] * 4
        msg.velocity = [self.vel] * 4
        msg.effort = []
        
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = WheelsOnlyTest()
    
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    import time
    time.sleep(0.5)
    
    try:
        while True:
            key = get_key()
            if key == 'w':
                node.vel = min(node.vel + 10.0, 50.0)
                print(f"\rVelocity: {node.vel:+6.1f} rad/s", end="", flush=True)
            elif key == 's':
                node.vel = max(node.vel - 10.0, -50.0)
                print(f"\rVelocity: {node.vel:+6.1f} rad/s", end="", flush=True)
            elif key == 'q':
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()