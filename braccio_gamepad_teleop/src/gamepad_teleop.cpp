#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <future>
#include <chrono>

#include <memory>
#include <cmath>
// Añadido para publicar directamente a ros2_control
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
// Para leer el estado del controller y obtener los nombres de joints que espera
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"

class GamepadTeleop : public rclcpp::Node, public std::enable_shared_from_this<GamepadTeleop>
{
public:
  GamepadTeleop() : Node("gamepad_teleop")
  {
    // Inicializar TF buffer
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    
    // Crear el subscriber primero
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&GamepadTeleop::joyCallback, this, std::placeholders::_1));

    // Publisher directo a controller (usar para evitar incompatibilidades de nombres)
    joint_cmd_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/position_trajectory_controller/joint_trajectory", 10);

    // Subscribir al estado del controller para capturar los joint_names que espera
    controller_state_sub = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        "/position_trajectory_controller/controller_state", 10,
        std::bind(&GamepadTeleop::controllerStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Gamepad teleop iniciando...");
  }

private:
  void controllerStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
    controller_joint_names = msg->joint_names;
    // Log corto con los nombres que el controller reporta
    std::string names;
    for (size_t i = 0; i < controller_joint_names.size(); ++i) {
      names += controller_joint_names[i];
      if (i + 1 < controller_joint_names.size()) names += ", ";
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Controller state joint_names (%zu): %s", controller_joint_names.size(), names.c_str());
  }
  
  void initializeMoveGroup()
  {
    try {
      move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          rclcpp::Node::shared_from_this(), "arm", tf_buffer, rclcpp::Duration::from_seconds(30.0));

      move_group->setMaxVelocityScalingFactor(0.2); // más seguro
      move_group->setMaxAccelerationScalingFactor(0.2);

      RCLCPP_INFO(this->get_logger(), "MoveGroup inicializado. Gamepad teleop listo");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error inicializando MoveGroup: %s", e.what());
    }
  }
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    if (!move_group) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                           "MoveGroup no inicializado; usando control directo de joints");
    }

    // Ejemplo: mapeo Xbox/PS
    double lx = joy->axes[0]; // stick izq, eje X
    double ly = joy->axes[1]; // stick izq, eje Y
    double ry = joy->axes[4]; // stick der, eje Y
    bool btn_a = joy->buttons[0]; // A / X en PS
    bool btn_b = joy->buttons[1]; // B / círculo en PS

    // Solo procesar si hay movimiento significativo
    double deadzone = 0.1;
    if (std::abs(lx) < deadzone && std::abs(ly) < deadzone && 
        std::abs(ry) < deadzone && !btn_a && !btn_b) {
      return; // No hay movimiento, no hacer nada
    }

    try {
      // Siempre usar movimiento directo de joints para mayor confiabilidad
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Usando modo de control directo de joints");
      sendDirectJointCommand(lx, ly, ry, btn_a, btn_b);
      return;
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Error en movimiento: %s", e.what());
    }
  }

  void sendDirectJointCommand(double lx, double ly, double ry, bool btn_a, bool btn_b)
  {
    // Usar estado local en lugar de depender de MoveIt para obtener joints
    // Inicializar con la postura actual en Gazebo (imagen): [9°, 45°, 180°, 180°, 9°]
    static std::vector<double> current_joint_values = {
        9.0 * M_PI / 180.0,    // joint_base  ~ 0.15708
        45.0 * M_PI / 180.0,   // joint_1     ~ 0.78540
        M_PI,                  // joint_2     ~ 3.14159
        M_PI,                  // joint_3     ~ 3.14159
        9.0 * M_PI / 180.0,    // joint_4     ~ 0.15708
        0.0                   // gripper
    };
    static bool initialized = false;
    
    if (!initialized) {
      RCLCPP_INFO(this->get_logger(), "Inicializando control directo de joints - posición inicial segura");
      initialized = true;
    }

    try {
      // Escala más pequeña para movimientos más suaves
      double joint_scale = 0.005; // radianes por comando (mucho más suave)
      
      // Aplicar movimientos incrementales usando nuestro estado local
      current_joint_values[0] += lx * joint_scale;  // Base rotation (horizontal)
      current_joint_values[1] += ly * joint_scale;  // Shoulder (up/down principal)
      current_joint_values[2] += ry * joint_scale;  // Elbow (extensión del brazo)
      
      // Botones para wrist
      if (btn_a) current_joint_values[3] += joint_scale * 0.5;  // Wrist vertical
      if (btn_b) current_joint_values[3] -= joint_scale * 0.5;  // Wrist vertical

      // Aplicar límites específicos del Braccio (en radianes)
      current_joint_values[0] = std::max(-3.0, std::min(3.0, current_joint_values[0]));    // Base: ±170°
      current_joint_values[1] = std::max(-1.57, std::min(3.14, current_joint_values[1]));  // Shoulder: -90° a 180°
      current_joint_values[2] = std::max(-3.14, std::min(3.14, current_joint_values[2]));  // Elbow: ±180°
      current_joint_values[3] = std::max(-3.14, std::min(3.14, current_joint_values[3]));  // Wrist: ±180°
      current_joint_values[4] = std::max(-3.14, std::min(3.14, current_joint_values[4]));  // Wrist rotation
      current_joint_values[5] = std::max(0.0, std::min(1.8, current_joint_values[5]));     // Gripper: 0-103°

      // Publicar directamente al controlador de posición usando SOLO las joints que el
      // controller espera (ver braccio_hardware/config/braccio_controllers.yml):
      // [joint_base, joint_1, joint_2, joint_3, joint_4]
      trajectory_msgs::msg::JointTrajectory traj;
      // Default: enviar en el orden que creemos correcto
      traj.joint_names = {"joint_base", "joint_1", "joint_2", "joint_3", "joint_4"};

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions = { current_joint_values[0], current_joint_values[1], current_joint_values[2], current_joint_values[3], current_joint_values[4] };
      pt.time_from_start = rclcpp::Duration::from_seconds(0.2);

      traj.points.push_back(pt);
      traj.header.stamp = this->get_clock()->now();

      // DEBUG: Log detallado antes de publicar
      auto vec_to_str = [](const std::vector<std::string> &v){ std::string s; for (size_t i=0;i<v.size();++i){ s+=v[i]; if(i+1<v.size()) s+", "; } return s; };
      auto nums_to_str = [](const std::vector<double> &v){ std::string s; char buf[64]; for (size_t i=0;i<v.size();++i){ snprintf(buf, sizeof(buf), "%.6f", v[i]); s+=buf; if(i+1<v.size()) s+", "; } return s; };

      RCLCPP_INFO(this->get_logger(), "Preparando a publicar JointTrajectory: joint_names=(%zu) [%s], positions_count=%zu [%s]",
                  traj.joint_names.size(), vec_to_str(traj.joint_names).c_str(), pt.positions.size(), nums_to_str(pt.positions).c_str());

      // Si conocemos qué espera el controller, mostrarlo y comparar
      if (!controller_joint_names.empty()) {
        RCLCPP_INFO(this->get_logger(), "Controller espera (%zu): [%s]",
                    controller_joint_names.size(), vec_to_str(controller_joint_names).c_str());

        if (controller_joint_names.size() != traj.joint_names.size()) {
          RCLCPP_ERROR(this->get_logger(), "Mismatch de número de joints: controller=%zu vs mensaje=%zu. No público para evitar error.",
                      controller_joint_names.size(), traj.joint_names.size());
          // Log de comparación por elemento
          RCLCPP_ERROR(this->get_logger(), "Controller joints: [%s]", vec_to_str(controller_joint_names).c_str());
          RCLCPP_ERROR(this->get_logger(), "Mensaje joints   : [%s]", vec_to_str(traj.joint_names).c_str());
          return; // evitar publicar mensaje erróneo
        }
        // Si mismos tamaños, comprobar nombres exactos
        bool all_match = true;
        for (size_t i = 0; i < controller_joint_names.size(); ++i) {
          if (controller_joint_names[i] != traj.joint_names[i]) { all_match = false; break; }
        }
        if (!all_match) {
          RCLCPP_ERROR(this->get_logger(), "Mismatch en orden/nombres de joints entre controller y mensaje. No publico.");
          RCLCPP_ERROR(this->get_logger(), "Controller joints: [%s]", vec_to_str(controller_joint_names).c_str());
          RCLCPP_ERROR(this->get_logger(), "Mensaje joints   : [%s]", vec_to_str(traj.joint_names).c_str());
          return;
        }
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Aún no se ha recibido controller_state; no podemos verificar nombres de joints.");
      }

      // Publicar al topic del controller
      if (joint_cmd_pub) {
        joint_cmd_pub->publish(traj);
        RCLCPP_DEBUG(this->get_logger(), "Publicado JointTrajectory directo (5 joints)");
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Publisher al controller no inicializado");
      }
 
     } catch (const std::exception& e) {
       RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Error en comando directo: %s", e.what());
     }
   }

   rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
   // Subscriber al estado del controller para comparar nombres de joints
   rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr controller_state_sub;
   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
   std::shared_ptr<tf2_ros::Buffer> tf_buffer;
   // rclcpp::TimerBase::SharedPtr init_timer;  // removed
   // Publisher directo al controlador de joints (solo para los 5 joints de posición)
   rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_pub;
   // Cache de los joint_names que el controller reporta
   std::vector<std::string> controller_joint_names;
 };

 int main(int argc, char **argv)
 {
   rclcpp::init(argc, argv);
   auto node = std::make_shared<GamepadTeleop>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
 }
