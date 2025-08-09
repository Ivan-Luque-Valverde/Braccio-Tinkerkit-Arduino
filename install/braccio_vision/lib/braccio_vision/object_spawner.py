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
        """Generar objetos de colores para pruebas"""
        
        # Objeto rojo (para detectar)
        red_cube_sdf = '''
        <?xml version="1.0"?>
        <sdf version="1.4">
          <model name="red_cube">
            <pose>0 0 0 0 0 0</pose>
            <static>false</static>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <box>
                    <size>0.05 0.05 0.05</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>0.05 0.05 0.05</size>
                  </box>
                </geometry>
                <material>
                  <ambient>1 0 0 1</ambient>
                  <diffuse>1 0 0 1</diffuse>
                </material>
              </visual>
              <inertial>
                <mass>0.1</mass>
                <inertia>
                  <ixx>0.000166</ixx>
                  <iyy>0.000166</iyy>
                  <izz>0.000166</izz>
                </inertia>
              </inertial>
            </link>
          </model>
        </sdf>
        '''
        
        # Posición del cubo rojo (en el área de trabajo del robot)
        pose = Pose()
        pose.position.x = 0.2    # Dentro del alcance del robot
        pose.position.y = 0.1    # Ligeramente desplazado en Y
        pose.position.z = 0.025  # Sobre el suelo (mitad del cubo de 0.05m)
        
        # Spawn del cubo rojo
        request = SpawnEntity.Request()
        request.name = 'red_cube_target'
        request.xml = red_cube_sdf
        request.robot_namespace = ''
        request.initial_pose = pose
        
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info('✅ Cubo rojo generado exitosamente')
        else:
            self.get_logger().error(f'❌ Error generando cubo rojo: {future.result().status_message}')
        
        # Esperar un poco antes del siguiente objeto
        time.sleep(1.0)
        
        # Objeto azul (distractor)
        blue_sphere_sdf = '''
        <?xml version="1.0"?>
        <sdf version="1.4">
          <model name="blue_sphere">
            <pose>0 0 0 0 0 0</pose>
            <static>false</static>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <sphere>
                    <radius>0.03</radius>
                  </sphere>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <sphere>
                    <radius>0.03</radius>
                  </sphere>
                </geometry>
                <material>
                  <ambient>0 0 1 1</ambient>
                  <diffuse>0 0 1 1</diffuse>
                </material>
              </visual>
              <inertial>
                <mass>0.05</mass>
                <inertia>
                  <ixx>0.00018</ixx>
                  <iyy>0.00018</iyy>
                  <izz>0.00018</izz>
                </inertia>
              </inertial>
            </link>
          </model>
        </sdf>
        '''
        
        # Posición de la esfera azul (objeto distractor)
        pose2 = Pose()
        pose2.position.x = 0.2   # Cerca pero diferente posición
        pose2.position.y = -0.2  # En el lado opuesto
        pose2.position.z = 0.03  # Sobre el suelo (radio de la esfera de 0.03m)
        
        # Spawn de la esfera azul
        request2 = SpawnEntity.Request()
        request2.name = 'blue_sphere_distractor'
        request2.xml = blue_sphere_sdf
        request2.robot_namespace = ''
        request2.initial_pose = pose2
        
        future2 = self.spawn_client.call_async(request2)
        rclpy.spin_until_future_complete(self, future2)
        
        if future2.result().success:
            self.get_logger().info('✅ Esfera azul generada exitosamente')
        else:
            self.get_logger().error(f'❌ Error generando esfera azul: {future2.result().status_message}')
        
        self.get_logger().info('🎯 Objetos de prueba generados. El sistema debería detectar el cubo rojo.')

def main(args=None):
    rclpy.init(args=args)
    
    spawner = ObjectSpawner()
    
    # Mantener el nodo activo brevemente para completar las operaciones
    time.sleep(3.0)
    
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
