#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class GamepadDirect : public rclcpp::Node
{
public:
  GamepadDirect() : Node("gamepad_direct")
  {
    // Crear subscriber del gamepad
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&GamepadDirect::joyCallback, this, std::placeholders::_1));

    // Publisher para comandos directos del robot
    joint_cmd_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/position_trajectory_controller/joint_trajectory", 10);

    // Estado inicial seguro del Braccio
    current_positions = {0.0, 1.57, -1.57, 0.0, 0.0, 0.0};
    
    RCLCPP_INFO(this->get_logger(), "🎮 Gamepad control directo iniciado");
    RCLCPP_INFO(this->get_logger(), "📡 Publicando comandos en: /position_trajectory_controller/joint_trajectory");
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    // Mapeo Xbox/PS
    double lx = joy->axes[0]; // stick izq, eje X
    double ly = joy->axes[1]; // stick izq, eje Y
    double ry = joy->axes[4]; // stick der, eje Y
    bool btn_a = joy->buttons[0]; // A / X en PS
    bool btn_b = joy->buttons[1]; // B / círculo en PS

    // Deadzone
    double deadzone = 0.1;
    if (std::abs(lx) < deadzone && std::abs(ly) < deadzone && 
        std::abs(ry) < deadzone && !btn_a && !btn_b) {
      return;
    }

    // Escala de movimiento (más pequeña para suavidad)
    double scale = 0.01; // radianes por comando

    // Aplicar movimientos incrementales
    current_positions[0] += lx * scale;  // joint_base (rotation horizontal)
    current_positions[1] += ly * scale;  // joint_1 (shoulder)
    current_positions[2] += ry * scale;  // joint_2 (elbow)
    
    if (btn_a) current_positions[3] += scale * 0.5;  // joint_3 (wrist up)
    if (btn_b) current_positions[3] -= scale * 0.5;  // joint_3 (wrist down)

    // Aplicar límites del Braccio
    current_positions[0] = std::max(-3.0, std::min(3.0, current_positions[0]));
    current_positions[1] = std::max(-1.57, std::min(3.14, current_positions[1]));
    current_positions[2] = std::max(-3.14, std::min(3.14, current_positions[2]));
    current_positions[3] = std::max(-3.14, std::min(3.14, current_positions[3]));
    current_positions[4] = std::max(-3.14, std::min(3.14, current_positions[4]));
    current_positions[5] = std::max(0.0, std::min(1.8, current_positions[5]));

    // Crear y enviar comando de trayectoria
    sendJointCommand();
    
    RCLCPP_DEBUG(this->get_logger(), "🎮 Comando: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                current_positions[0], current_positions[1], current_positions[2],
                current_positions[3], current_positions[4], current_positions[5]);
  }

  void sendJointCommand()
  {
    auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    
    // Nombres de los joints del Braccio (NOMBRES REALES del controlador)
    trajectory_msg.joint_names = {
      "joint_base",
      "joint_1", 
      "joint_2",
      "joint_3",
      "joint_4",
      "right_gripper_joint"
    };

    // Crear punto de trayectoria
    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    point.positions = current_positions;
    point.time_from_start = rclcpp::Duration::from_seconds(0.2); // 200ms para llegar

    trajectory_msg.points.push_back(point);
    trajectory_msg.header.stamp = this->get_clock()->now();

    // Publicar comando
    joint_cmd_pub->publish(trajectory_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_pub;
  std::vector<double> current_positions;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GamepadDirect>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
