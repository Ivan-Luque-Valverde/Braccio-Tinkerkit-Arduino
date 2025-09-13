#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import os
import sys
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class SequenceSelector(Node):
    def __init__(self):
        super().__init__('sequence_selector')
        
        # Cargar configuración
        self.load_config()
        
        # Publishers
        self.arm_publisher = self.create_publisher(
            JointTrajectory,
            '/position_trajectory_controller/joint_trajectory',
            10
        )
        
        self.gripper_publisher = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info('🤖 Selector de Secuencias iniciado')
        self.get_logger().info(f'📋 {len(self.config.get("sequences", {}))} secuencias disponibles')
        
        # Dar tiempo para que se inicialicen los publishers
        time.sleep(1.0)

    def load_config(self):
        """Cargar configuración desde archivo YAML"""
        try:
            config_path = '/home/ivan/Escritorio/Braccio-Tinkerkit-Arduino/braccio_moveit_config/config/pick_and_place_config.yaml'
            
            with open(config_path, 'r') as file:
                self.config = yaml.safe_load(file)
            
            self.get_logger().info(f'✅ Configuración cargada desde: {config_path}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error cargando configuración: {str(e)}')
            sys.exit(1)

    def show_available_sequences(self):
        """Mostrar todas las secuencias disponibles"""
        sequences = self.config.get('sequences', {})
        
        print("\n" + "="*50)
        print("🎭 SECUENCIAS DISPONIBLES")
        print("="*50)
        
        for i, (seq_name, sequence) in enumerate(sequences.items(), 1):
            print(f"{i}. {seq_name}")
            print(f"   📝 {len(sequence)} pasos:")
            for step in sequence:
                action = step.get('action', 'unknown')
                description = step.get('description', 'Sin descripción')
                print(f"      • {action}: {description}")
            print()
        
        print("0. ❌ Salir")
        print("="*50)

    def create_arm_trajectory(self, positions, duration_sec):
        """Crea una trayectoria para el brazo"""
        traj = JointTrajectory()
        traj.joint_names = [
            "joint_base",
            "joint_1",
            "joint_2", 
            "joint_3",
            "joint_4"
        ]
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        traj.points.append(point)
        
        return traj

    def create_gripper_trajectory(self, position, duration_sec):
        """Crea una trayectoria para el gripper"""
        traj = JointTrajectory()
        traj.joint_names = ["right_gripper_joint"]
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        traj.points.append(point)
        
        return traj

    def move_joints(self, target_name, duration=3.0):
        """Mover articulaciones a una posición específica"""
        joint_positions = self.config.get('joint_positions', {})
        
        if target_name not in joint_positions:
            self.get_logger().error(f'❌ Posición "{target_name}" no encontrada')
            return False
        
        target_position = joint_positions[target_name]
        
        if duration is None:
            duration = self.config.get('movement', {}).get('default_duration', 3.0)
        
        # Crear trayectoria usando el método correcto
        traj = self.create_arm_trajectory(target_position, duration)
        traj.header.stamp = self.get_clock().now().to_msg()
        
        # Publicar
        self.arm_publisher.publish(traj)
        self.get_logger().info(f'🤖 Moviendo a: {target_name} ({duration}s)')
        self.get_logger().info(f'📐 Posiciones: {target_position}')
        
        # Esperar a que termine el movimiento
        time.sleep(duration + 0.5)
        return True

    def control_gripper(self, action):
        """Controlar el gripper (abrir/cerrar)"""
        gripper_config = self.config.get('gripper', {})
        
        if action == 'open_gripper':
            position = gripper_config.get('open_position', 0.0)
            duration = gripper_config.get('open_time', 3.0)
            self.get_logger().info('🤏 Abriendo gripper...')
        elif action == 'close_gripper':
            position = gripper_config.get('closed_position', 0.75)
            duration = gripper_config.get('close_time', 3.0)
            self.get_logger().info('🤏 Cerrando gripper...')
        else:
            self.get_logger().error(f'❌ Acción de gripper desconocida: {action}')
            return False
        
        # Crear trayectoria para gripper usando el método correcto
        traj = self.create_gripper_trajectory(position, duration)
        traj.header.stamp = self.get_clock().now().to_msg()
        
        # Publicar
        self.gripper_publisher.publish(traj)
        
        # Esperar a que termine el movimiento
        time.sleep(duration + 0.5)
        return True

    def execute_sequence(self, sequence_name):
        """Ejecutar una secuencia específica"""
        sequences = self.config.get('sequences', {})
        
        if sequence_name not in sequences:
            self.get_logger().error(f'❌ Secuencia "{sequence_name}" no encontrada')
            return False
        
        sequence = sequences[sequence_name]
        
        print(f"\n🎭 EJECUTANDO SECUENCIA: {sequence_name}")
        print("="*50)
        
        for i, step in enumerate(sequence, 1):
            action = step.get('action', 'unknown')
            description = step.get('description', 'Sin descripción')
            target = step.get('target', None)
            
            print(f"📍 Paso {i}/{len(sequence)}: {description}")
            
            success = False
            
            if action == 'move_joints' and target:
                success = self.move_joints(target)
            elif action in ['open_gripper', 'close_gripper']:
                success = self.control_gripper(action)
            else:
                self.get_logger().warning(f'⚠️  Acción desconocida: {action}')
                continue
            
            if not success:
                self.get_logger().error(f'❌ Error en paso {i}: {description}')
                return False
            
            print(f"✅ Completado: {description}")
        
        print(f"\n🎉 SECUENCIA '{sequence_name}' COMPLETADA EXITOSAMENTE!")
        print("="*50)
        return True

    def run_interactive_mode(self):
        """Ejecutar modo interactivo para selección de secuencias"""
        sequences = self.config.get('sequences', {})
        sequence_list = list(sequences.keys())
        
        while True:
            self.show_available_sequences()
            
            try:
                choice = input("\n🎯 Selecciona una secuencia (número): ").strip()
                
                if choice == '0':
                    print("👋 ¡Hasta luego!")
                    break
                
                choice_num = int(choice)
                
                if 1 <= choice_num <= len(sequence_list):
                    selected_sequence = sequence_list[choice_num - 1]
                    
                    print(f"\n🚀 ¿Ejecutar secuencia '{selected_sequence}'? (s/n): ", end="")
                    confirm = input().strip().lower()
                    
                    if confirm in ['s', 'si', 'y', 'yes']:
                        self.execute_sequence(selected_sequence)
                    else:
                        print("❌ Ejecución cancelada")
                else:
                    print("❌ Número inválido. Intenta de nuevo.")
                    
            except ValueError:
                print("❌ Por favor ingresa un número válido.")
            except KeyboardInterrupt:
                print("\n👋 ¡Hasta luego!")
                break
            
            input("\n⏸️  Presiona Enter para continuar...")

def main():
    rclpy.init()
    
    try:
        selector = SequenceSelector()
        selector.run_interactive_mode()
        
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
