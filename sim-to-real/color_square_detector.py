#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorSquareDetector(Node):
    """Detecta cuadrados de colores en la imagen y publica una imagen de debug
    y posiciones estimadas en el frame 'camera_link'.
    - Publica: /vision/debug_image (Image), /vision/detections (PoseStamped para cada detección)
    """
    def __init__(self, topic_sub='/overhead_camera/image_raw'):
        super().__init__('color_square_detector')
        self.sub = self.create_subscription(Image, topic_sub, self.img_cb, 10)
        self.pub_debug = self.create_publisher(Image, '/vision/debug_image', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/vision/detections', 10)
        self.bridge = CvBridge()
        self.get_logger().info('Detector de cuadrados de color iniciado')

        # Parámetros simples de cámara (ajustar según calibración)
        self.declare_parameter('fx', 600.0)
        self.declare_parameter('fy', 600.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value

        # Altura estimada del plano de trabajo en frame world (m)
        self.declare_parameter('object_z', 0.015)
        self.object_z = self.get_parameter('object_z').value

    def img_cb(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        vis = img.copy()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Solo detectar color rojo (sin restricción de forma). Rango HSV para rojo
        red_ranges = [((0, 100, 50), (10, 255, 255)), ((170, 100, 50), (180, 255, 255))]

        mask = None
        for lo, hi in red_ranges:
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)

        # Filtrar ruido suavemente
        mask = cv2.GaussianBlur(mask, (7,7), 0)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # Contornos: no exigimos forma, sólo tamaño mínimo para evitar ruido
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 200:  # umbral muy bajo para filtrar ruido
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w/2.0
            cy = y + h/2.0
            detections.append(('red', cx, cy, area, x, y, w, h))
            # Dibujar caja y etiqueta en debug
            cv2.rectangle(vis, (x, y), (x+w, y+h), (0,0,255), 2)
            cv2.putText(vis, 'red', (x, y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
            # Dibujar centro del bounding box (círculo y cruz)
            cx_i = int(cx)
            cy_i = int(cy)
            cv2.circle(vis, (cx_i, cy_i), 5, (0,255,255), -1)
            cv2.line(vis, (cx_i-10, cy_i), (cx_i+10, cy_i), (0,255,255), 2)
            cv2.line(vis, (cx_i, cy_i-10), (cx_i, cy_i+10), (0,255,255), 2)
            # Opcional: mostrar coordenadas de píxel junto al centro
            coord_text = f'({cx_i},{cy_i})'
            cv2.putText(vis, coord_text, (cx_i+8, cy_i-8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
                        
        # LOG: cuántas detecciones hay en este frame
        try:
            self.get_logger().info(f"Detecciones en frame: {len(detections)}")
        except Exception:
            pass

        # Si no hay detecciones, dibujar un marco y texto claramente visible
        if len(detections) == 0:
            h_img, w_img = vis.shape[:2]
            # marco rojo alrededor de la imagen para diferenciar
            cv2.rectangle(vis, (2,2), (w_img-2, h_img-2), (0,0,255), 4)
            cv2.putText(vis, 'NO DETECTIONS', (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 3)
        # Publicar imagen debug
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            debug_msg.header = msg.header
            self.pub_debug.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Error publicando debug image: {e}')

        # Publicar detecciones como PoseStamped (en camera_link)
        for det in detections:
            # det = ('red', cx, cy, area, x, y, w, h)
            _, px, py, area, bx, by, bw, bh = det
            Z = self.object_z
            Xc = (px - self.cx) * Z / self.fx
            Yc = (py - self.cy) * Z / self.fy
            pose = PoseStamped()
            pose.header = msg.header
            pose.header.frame_id = 'camera_link'
            pose.pose.position.x = float(Xc)
            pose.pose.position.y = float(Yc)
            pose.pose.position.z = float(Z)
            # Publicar la pose del centro del bounding box
            self.pub_pose.publish(pose)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColorSquareDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
