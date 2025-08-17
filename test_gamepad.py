#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class GamepadMonitor(Node):
    def __init__(self):
        super().__init__('gamepad_monitor')
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info('🎮 Monitoring gamepad input...')

    def joy_callback(self, msg):
        lx = msg.axes[0]
        ly = msg.axes[1] 
        ry = msg.axes[4]
        btn_a = msg.buttons[0]
        btn_b = msg.buttons[1]
        
        # Show significant values only
        if abs(lx) > 0.05 or abs(ly) > 0.05 or abs(ry) > 0.05 or btn_a or btn_b:
            self.get_logger().info(f'LX: {lx:.3f}, LY: {ly:.3f}, RY: {ry:.3f}, A: {btn_a}, B: {btn_b}')

def main():
    rclpy.init()
    node = GamepadMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
