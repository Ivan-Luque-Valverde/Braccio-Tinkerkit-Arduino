#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('sim_camera_viewer')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/overhead_camera/image_raw', self.cb_img, 10)
        self.sub_debug = self.create_subscription(Image, '/vision/debug_image', self.cb_debug, 10)
        self.current = None
        self.debug = None
        self.timer = self.create_timer(0.05, self.show)
        cv2.namedWindow('Raw', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Debug', cv2.WINDOW_NORMAL)
        self.get_logger().info('Viewer iniciado. Esperando frames...')

    def cb_img(self, msg):
        try:
            self.current = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')

    def cb_debug(self, msg):
        try:
            self.debug = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error debug: {e}')

    def show(self):
        if self.current is not None:
            cv2.imshow('Raw', self.current)
        if self.debug is not None:
            cv2.imshow('Debug', self.debug)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
