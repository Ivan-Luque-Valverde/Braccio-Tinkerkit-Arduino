#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include <chrono>
#include <cmath>

class JointTester : public rclcpp::Node
{
public:
  JointTester() : Node("joint_tester")
  {
    // Publisher para comandos
    joint_cmd_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/position_trajectory_controller/joint_trajectory", 10);

    // Subscriber para estado del controller
    controller_state_sub = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        "/position_trajectory_controller/controller_state", 10,
        std::bind(&JointTester::controllerStateCallback, this, std::placeholders::_1));

    // Timer para ejecutar tests cada 3 segundos
    test_timer = this->create_wall_timer(
        std::chrono::seconds(3),
        std::bind(&JointTester::executeTest, this));

    RCLCPP_INFO(this->get_logger(), "=== JOINT TESTER INICIADO ===");
    RCLCPP_INFO(this->get_logger(), "Esperando estado del controller...");
    
    // Posición inicial segura (en radianes) - SINCRONIZADA con MoveIt SRDF
    // MoveIt HOME: joint_base=0, joint_1=1.55, joint_2=0, joint_3=0, joint_4=0
    home_position = {0.0, 1.55, 0.0, 0.0, 0.0}; // base, shoulder, elbow, wrist, wrist_rot
    current_test_joint = 0;
    test_direction = 1;
    use_current_as_home = true; // Usar posición actual del robot como HOME
    first_cycle = true;
    
    // Incrementos pequeños para cada joint (en radianes)
    test_increments = {0.1, 0.1, 0.1, 0.1, 0.1}; // ~5.7 grados cada uno
  }

private:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_pub;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr controller_state_sub;
  rclcpp::TimerBase::SharedPtr test_timer;
  
  std::vector<std::string> controller_joint_names;
  std::vector<double> home_position;
  std::vector<double> test_increments;
  std::vector<double> current_position;
  
  int current_test_joint;
  int test_direction; // 1 o -1
  int test_cycle = 0;
  bool use_current_as_home;
  bool first_cycle;

  void controllerStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
    controller_joint_names = msg->joint_names;
    current_position = msg->actual.positions;
    
    // En el primer callback, usar la posición actual como HOME si está habilitado
    if (use_current_as_home && first_cycle && current_position.size() >= 5) {
      home_position = {current_position[0], current_position[1], current_position[2], 
                      current_position[3], current_position[4]};
      RCLCPP_INFO(this->get_logger(), "USANDO POSICIÓN ACTUAL COMO HOME: %s", 
                  positionsToStringDegrees(home_position).c_str());
      first_cycle = false;
    }
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Controller joints (%zu): %s", 
                         controller_joint_names.size(),
                         jointsToString(controller_joint_names).c_str());
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Posiciones actuales: %s", 
                         positionsToString(current_position).c_str());
  }

  void executeTest()
  {
    if (controller_joint_names.empty()) {
      RCLCPP_WARN(this->get_logger(), "Esperando estado del controller...");
      return;
    }

    if (controller_joint_names.size() != 5) {
      RCLCPP_ERROR(this->get_logger(), "Expected 5 joints, got %zu", controller_joint_names.size());
      return;
    }

    test_cycle++;
    
    // Cada 10 ciclos, ir a home position
    if (test_cycle % 10 == 1) {
      RCLCPP_INFO(this->get_logger(), "\n=== CICLO %d: VOLVIENDO A HOME POSITION ===", test_cycle);
      RCLCPP_INFO(this->get_logger(), "HOME en grados: %s", positionsToStringDegrees(home_position).c_str());
      sendJointCommand(home_position, "VOLVER A HOME");
      return;
    }

    // Test individual de cada joint
    std::vector<double> test_position = home_position;
    
    // Aplicar pequeño movimiento al joint actual
    double increment = test_increments[current_test_joint] * test_direction;
    test_position[current_test_joint] += increment;
    
    // Aplicar límites de seguridad
    applyJointLimits(test_position);
    
    std::string joint_name = (current_test_joint < controller_joint_names.size()) ? 
                            controller_joint_names[current_test_joint] : 
                            "joint_" + std::to_string(current_test_joint);
    
    RCLCPP_INFO(this->get_logger(), "\n=== CICLO %d: TESTANDO %s ===", 
                test_cycle, joint_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Incremento: %.3f radianes (%.1f grados) en dirección %s",
                increment, increment * 180.0/M_PI, 
                (test_direction > 0) ? "POSITIVA" : "NEGATIVA");
    
    sendJointCommand(test_position, "TEST " + joint_name);
    
    // Cambiar a siguiente joint o cambiar dirección
    test_direction *= -1;
    if (test_direction == 1) {
      current_test_joint = (current_test_joint + 1) % 5;
    }
  }

  void sendJointCommand(const std::vector<double>& positions, const std::string& description)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = controller_joint_names;
    
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = positions;
    pt.time_from_start = rclcpp::Duration::from_seconds(2.0); // 2 segundos para llegar
    
    traj.points.push_back(pt);
    traj.header.stamp = this->get_clock()->now();
    
    RCLCPP_INFO(this->get_logger(), "[ENVIANDO] %s", description.c_str());
    RCLCPP_INFO(this->get_logger(), "Joints: %s", jointsToString(traj.joint_names).c_str());
    RCLCPP_INFO(this->get_logger(), "Posiciones (rad): %s", positionsToString(positions).c_str());
    RCLCPP_INFO(this->get_logger(), "Posiciones (deg): %s", positionsToStringDegrees(positions).c_str());
    
    if (joint_cmd_pub) {
      joint_cmd_pub->publish(traj);
      RCLCPP_INFO(this->get_logger(), "[OK] Comando publicado");
    } else {
      RCLCPP_ERROR(this->get_logger(), "[ERROR] Publisher no inicializado");
    }
    
    RCLCPP_INFO(this->get_logger(), "---");
  }

  void applyJointLimits(std::vector<double>& positions)
  {
    if (positions.size() >= 5) {
      positions[0] = std::max(-3.0, std::min(3.0, positions[0]));    // Base: ±170°
      positions[1] = std::max(-1.57, std::min(3.14, positions[1]));  // Shoulder: -90° a 180°
      positions[2] = std::max(-3.14, std::min(3.14, positions[2]));  // Elbow: ±180°
      positions[3] = std::max(-3.14, std::min(3.14, positions[3]));  // Wrist: ±180°
      positions[4] = std::max(-3.14, std::min(3.14, positions[4]));  // Wrist rotation: ±180°
    }
  }

  std::string jointsToString(const std::vector<std::string>& joints)
  {
    std::string result = "[";
    for (size_t i = 0; i < joints.size(); ++i) {
      result += joints[i];
      if (i + 1 < joints.size()) result += ", ";
    }
    result += "]";
    return result;
  }

  std::string positionsToString(const std::vector<double>& positions)
  {
    std::string result = "[";
    for (size_t i = 0; i < positions.size(); ++i) {
      char buf[32];
      snprintf(buf, sizeof(buf), "%.4f", positions[i]);
      result += buf;
      if (i + 1 < positions.size()) result += ", ";
    }
    result += "]";
    return result;
  }

  std::string positionsToStringDegrees(const std::vector<double>& positions)
  {
    std::string result = "[";
    for (size_t i = 0; i < positions.size(); ++i) {
      char buf[32];
      snprintf(buf, sizeof(buf), "%.1f°", positions[i] * 180.0/M_PI);
      result += buf;
      if (i + 1 < positions.size()) result += ", ";
    }
    result += "]";
    return result;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTester>();
  
  RCLCPP_INFO(node->get_logger(), "=== INSTRUCCIONES ===");
  RCLCPP_INFO(node->get_logger(), "1. Este script enviará comandos cada 3 segundos");
  RCLCPP_INFO(node->get_logger(), "2. Primero irá a posición HOME");
  RCLCPP_INFO(node->get_logger(), "3. Luego testará cada joint individualmente");
  RCLCPP_INFO(node->get_logger(), "4. Observa si cada joint se mueve cuando debería");
  RCLCPP_INFO(node->get_logger(), "5. Usa Ctrl+C para parar");
  RCLCPP_INFO(node->get_logger(), "===================");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
