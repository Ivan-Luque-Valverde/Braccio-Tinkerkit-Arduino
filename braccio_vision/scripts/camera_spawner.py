#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import time

class CameraSpawner(Node):
    def __init__(self):
        super().__init__('camera_spawner')
        
        # Cliente para spawn de entidades en Gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        # Esperar a que el servicio esté disponible
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio de spawn...')
        
        self.get_logger().info('📷 Generador de cámara iniciado')
        
        # Crear cámara cenital
        self.spawn_overhead_camera()
        
    def spawn_overhead_camera(self):
        """Generar cámara cenital en la simulación"""
        
        # SDF de la cámara cenital
        camera_sdf = '''
        <?xml version="1.0"?>
        <sdf version="1.4">
          <model name="overhead_camera">
            <static>true</static>
            <pose>0.4 0 0.6 0 1.5708 0</pose>
            
            <link name="camera_link">
              
              <!-- Cuerpo de la cámara (visual) -->
              <visual name="camera_visual">
                <geometry>
                  <box>
                    <size>0.05 0.05 0.1</size>
                  </box>
                </geometry>
                <material>
                  <ambient>0.2 0.2 0.2 1</ambient>
                  <diffuse>0.2 0.2 0.2 1</diffuse>
                </material>
              </visual>
              
              <!-- Colisión de la cámara -->
              <collision name="camera_collision">
                <geometry>
                  <box>
                    <size>0.05 0.05 0.1</size>
                  </box>
                </geometry>
              </collision>
              
              <!-- Sensor de cámara -->
              <sensor type="camera" name="overhead_camera_sensor">
                <update_rate>30.0</update_rate>
                <visualize>true</visualize>
                
                <camera name="overhead_camera">
                  <horizontal_fov>1.047</horizontal_fov>
                  <image>
                    <width>640</width>
                    <height>480</height>
                    <format>R8G8B8</format>
                  </image>
                  <clip>
                    <near>0.1</near>
                    <far>100</far>
                  </clip>
                  <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.007</stddev>
                  </noise>
                </camera>
                
                <!-- Plugin de ROS2 para la cámara -->
                <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
                  <ros>
                    <namespace>/</namespace>
                    <remapping>image_raw:=overhead_camera/image_raw</remapping>
                    <remapping>camera_info:=overhead_camera/camera_info</remapping>
                  </ros>
                  <camera_name>overhead_camera</camera_name>
                  <frame_name>overhead_camera_link</frame_name>
                </plugin>
                
              </sensor>
            </link>
          </model>
        </sdf>
        '''
        
        # Posición de la cámara (cenital sobre el área de trabajo)
        pose = Pose()
        pose.position.x = 0.4    # Sobre el área de trabajo del robot
        pose.position.y = 0.0    # Centrado en Y
        pose.position.z = 0.6    # Altura bajada para vista más cercana (era 1.0)
        
        # Orientación: cámara mirando directamente hacia abajo (plano X-Y)
        # Para mirar hacia abajo: rotación de 180° en eje X (roll) o 180° en Z (yaw)
        # Quaternion para rotación de 180° en Z: x=0, y=0, z=1, w=0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 1.0
        pose.orientation.w = 0.0
        
        # Spawn de la cámara
        request = SpawnEntity.Request()
        request.name = 'overhead_camera'
        request.xml = camera_sdf
        request.robot_namespace = ''
        request.initial_pose = pose
        
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info('✅ Cámara cenital generada exitosamente')
            self.get_logger().info('📸 Topic de imagen: /overhead_camera/image_raw')
        else:
            self.get_logger().error(f'❌ Error generando cámara: {future.result().status_message}')

def main(args=None):
    rclpy.init(args=args)
    
    spawner = CameraSpawner()
    
    # Mantener el nodo activo brevemente para completar las operaciones
    time.sleep(3.0)
    
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
