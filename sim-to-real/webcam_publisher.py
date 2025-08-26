#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self, topic='/overhead_camera/image_raw', device=2, fps=15):
        super().__init__('webcam_publisher')
        self.pub = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(device)
        # Intentar configurar resolución (suave si no es soportado)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.rate = 1.0 / max(1.0, fps)
        self.timer = self.create_timer(self.rate, self.timer_cb)
        self.get_logger().info(f'Publicando webcam en: {topic} (device={device})')

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('No se pudo leer frame de la webcam')
            return
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo/publicando imagen: {e}')

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
