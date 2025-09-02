#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import math


class WebcamPublisher(Node):
    def __init__(self, topic='/overhead_camera/image_raw', device=2, fps=15):
        super().__init__('webcam_publisher')
        self.pub = self.create_publisher(Image, topic, 10)
        self.caminfo_pub = self.create_publisher(CameraInfo, '/overhead_camera/camera_info', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(device)

        # Try to set resolution (will be ignored if unsupported)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.rate = 1.0 / max(1.0, fps)
        self.timer = self.create_timer(self.rate, self.timer_cb)
        self.get_logger().info(f'Publishing webcam on: {topic} (device={device})')

        # Prepare CameraInfo from hfov (80 deg ~ 1.3962634 rad)
        self.width = 640
        self.height = 480
        self.hfov = 1.3962634
        self.fx = self.width / (2.0 * math.tan(self.hfov / 2.0))
        self.fy = self.fx
        self.cx = float(self.width) / 2.0
        self.cy = float(self.height) / 2.0

        self._camera_info = CameraInfo()
        self._camera_info.width = int(self.width)
        self._camera_info.height = int(self.height)
        self._camera_info.distortion_model = 'plumb_bob'
        self._camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._camera_info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        self._camera_info.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Could not read frame from webcam')
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'

            # Publish CameraInfo alongside the image
            ci = CameraInfo()
            ci.header.stamp = msg.header.stamp
            ci.header.frame_id = 'camera_link'
            ci.width = self._camera_info.width
            ci.height = self._camera_info.height
            ci.distortion_model = self._camera_info.distortion_model
            ci.d = list(self._camera_info.d)
            ci.k = list(self._camera_info.k)
            ci.p = list(self._camera_info.p)

            self.caminfo_pub.publish(ci)
            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error converting/publishing image: {e}')

    def destroy_node(self):
        try:
            self.cap.release()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
