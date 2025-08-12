#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
import time

class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')
        
        # Cliente para spawn de entidades en Gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # Esperar a que los servicios estén disponibles
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio de spawn...')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio de delete...')
        
        self.get_logger().info('Generador de objetos iniciado')
        
        # Crear objetos de prueba
        success = self.spawn_test_objects()
        if success:
            self.get_logger().info('🎉 Todos los objetos se generaron correctamente')
        else:
            self.get_logger().warn('⚠️  Algunos objetos no se pudieron generar')
    
    def delete_existing_model(self, model_name):
        """Eliminar modelo existente si existe"""
        request = DeleteEntity.Request()
        request.name = model_name
        future = self.delete_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        # No importa si falla (modelo no existe)
        
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
            # Cubos verdes dentro del workspace
             {"name": "green_cube1", "x": 0.35, "y": 0.05, "z": 0.025, "color": (0, 1, 0)},   # verde - POSICIÓN CORREGIDA
             {"name": "green_cube2", "x": 0.28, "y": 0.18, "z": 0.025, "color": (0, 1, 0)},   # verde - SEGURA
             {"name": "green_cube3", "x": 0.28, "y": -0.15, "z": 0.025, "color": (0, 1, 0)},   # verde - POSICIÓN CORREGIDA
        ]

        successful_spawns = 0
        total_cubes = len(cubes)
        
        for cube in cubes:
            # Eliminar modelo existente primero
            self.get_logger().info(f'🔄 Procesando cubo: {cube["name"]}')
            self.delete_existing_model(cube["name"])
            time.sleep(0.2)
            
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
                successful_spawns += 1
                self.get_logger().info(f'✅ Cubo {cube["name"]} generado exitosamente en ({cube["x"]}, {cube["y"]}, {cube["z"]})')
            else:
                self.get_logger().error(f'❌ Error generando cubo {cube["name"]}: {future.result().status_message}')
            time.sleep(0.5)
        
        self.get_logger().info(f'🎯 Generación completada: {successful_spawns}/{total_cubes} cubos generados exitosamente.')
        return successful_spawns == total_cubes

def main(args=None):
    rclpy.init(args=args)
    
    spawner = ObjectSpawner()
    
    # Mantener el nodo activo más tiempo para launch files
    time.sleep(3.0)  # Tiempo suficiente para completar todas las operaciones
    
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
