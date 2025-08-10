#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import time

class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')
        
        # Cliente para spawn de entidades en Gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        # Esperar a que el servicio esté disponible
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio de spawn...')
        
        self.get_logger().info('Generador de objetos iniciado')
        
        # Crear objetos de prueba
        self.spawn_test_objects()
        
    def spawn_test_objects(self):
        """Generar 4 cubos pequeños en las esquinas del workspace"""
        # Definición del cubo pequeño (5cm)
        cube_sdf_template = '''
        <?xml version="1.0"?>
        <sdf version="1.4">
          <model name="{name}">
            <pose>0 0 0 0 0 0</pose>
            <static>false</static>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <box>
                    <size>0.03 0.03 0.03</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>0.03 0.03 0.03</size>
                  </box>
                </geometry>
                <material>
                  <ambient>{r} {g} {b} 1</ambient>
                  <diffuse>{r} {g} {b} 1</diffuse>
                </material>
              </visual>
              <inertial>
                <mass>0.05</mass>
                <inertia>
                  <ixx>0.00004</ixx>
                  <iyy>0.00004</iyy>
                  <izz>0.00004</izz>
                </inertia>
              </inertial>
            </link>
          </model>
        </sdf>
        '''

        # Coordenadas de las esquinas (ajusta si tu workspace es diferente)
        cubes = [
            # Esquinas más espaciadas para mejor calibración
            {"name": "corner1", "x": -0.35, "y": -0.25, "z": 0.025, "color": (1, 0, 0)},   # rojo
            {"name": "corner2", "x": 0.35, "y": -0.25, "z": 0.025, "color": (1, 0, 0)},   # rojo  
            {"name": "corner3", "x": 0.35, "y": 0.25,  "z": 0.025, "color": (1, 0, 0)},   # rojo
            {"name": "corner4", "x": -0.35, "y": 0.25,  "z": 0.025, "color": (1, 0, 0)},   # rojo
            # Cubo verde dentro del workspace
             {"name": "green_cube", "x": 0.32, "y": 0.1, "z": 0.025, "color": (0, 1, 0)},   # verde
        ]

        for cube in cubes:
            sdf = cube_sdf_template.format(
                name=cube["name"],
                r=cube["color"][0],
                g=cube["color"][1],
                b=cube["color"][2]
            )
            pose = Pose()
            pose.position.x = cube["x"]
            pose.position.y = cube["y"]
            pose.position.z = cube["z"]
            request = SpawnEntity.Request()
            request.name = cube["name"]
            request.xml = sdf
            request.robot_namespace = ''
            request.initial_pose = pose
            future = self.spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result().success:
                self.get_logger().info(f'✅ Cubo {cube["name"]} generado exitosamente')
            else:
                self.get_logger().error(f'❌ Error generando cubo {cube["name"]}: {future.result().status_message}')
            time.sleep(0.5)
        self.get_logger().info('🎯 Cubos de esquina y cubo verde generados. Puedes usarlos para calibrar y para pick and place.')

def main(args=None):
    rclpy.init(args=args)
    
    spawner = ObjectSpawner()
    
    # Mantener el nodo activo brevemente para completar las operaciones
    time.sleep(3.0)
    
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
