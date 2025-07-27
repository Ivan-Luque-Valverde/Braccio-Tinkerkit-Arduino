#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator')
        self.get_logger().info('📐 Camera Calibrator - En desarrollo')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraCalibrator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
