#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <array>
#include <algorithm>

class GamepadTeleopSimple : public rclcpp::Node
{
public:
    GamepadTeleopSimple() : Node("gamepad_teleop_simple")
    {
        // Posiciones iniciales basadas en el estado real del robot en Gazebo
        current_joint_positions_ = {1.573, 0.763, 3.142, 3.127, 1.572, 0.0};

        RCLCPP_INFO(this->get_logger(), "🎮 Gamepad Teleop Simple iniciado");
        RCLCPP_INFO(this->get_logger(), "📐 Posiciones iniciales (estado real): [%.3f, %.3f, %.3f, %.3f, %.3f]", 
                   current_joint_positions_[0], current_joint_positions_[1], current_joint_positions_[2], 
                   current_joint_positions_[3], current_joint_positions_[4]);

        // Subscriber para mensajes de joystick
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&GamepadTeleopSimple::joy_callback, this, std::placeholders::_1));

        // Publisher directo usando el mismo método que pick_and_place
        trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/position_trajectory_controller/joint_trajectory", 10);



        RCLCPP_INFO(this->get_logger(), "✅ Publisher configurado usando método pick_and_place");
        RCLCPP_INFO(this->get_logger(), "🎮 Usa el stick izquierdo para controlar base y joint_1");
        RCLCPP_INFO(this->get_logger(), "🎮 Usa el stick derecho para controlar joint_2 y joint_3"); 
        RCLCPP_INFO(this->get_logger(), "🎮 Usa L1/R1 para controlar joint_4");
        RCLCPP_INFO(this->get_logger(), "📍 Modo: Incremental desde estado real del robot");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Deadzone para evitar drift
        const double deadzone = 0.1;

        // Mapeo de controles (PlayStation/Xbox layout)
        double left_x = std::abs(msg->axes[0]) > deadzone ? msg->axes[0] : 0.0;  // joint_base
        double left_y = std::abs(msg->axes[1]) > deadzone ? msg->axes[1] : 0.0;  // joint_1
        double right_x = std::abs(msg->axes[3]) > deadzone ? msg->axes[3] : 0.0; // joint_2
        double right_y = std::abs(msg->axes[4]) > deadzone ? msg->axes[4] : 0.0; // joint_3

        // Botones para joint_4
        double joint_4_input = 0.0;
        if (msg->buttons.size() > 5) {
            if (msg->buttons[4]) joint_4_input = 1.0;  // L1 -> positivo
            if (msg->buttons[5]) joint_4_input = -1.0; // R1 -> negativo
        }
        // Botones para gripper (right_gripper_joint)
        double gripper_input = 0.0;
        if (msg->buttons.size() > 7) {
            if (msg->buttons[6]) gripper_input = 1.0;  // L2 -> abrir
            if (msg->buttons[7]) gripper_input = -1.0; // R2 -> cerrar
        }
        // Solo procesar si hay algún input
        if (std::abs(left_x) > 0 || std::abs(left_y) > 0 || 
            std::abs(right_x) > 0 || std::abs(right_y) > 0 || 
            std::abs(joint_4_input) > 0 || std::abs(gripper_input) > 0) {
            // Factor de velocidad ajustado para suavidad y rapidez
            const double velocity_factor = 0.005; // Incremento menor para suavidad
            // Aplicar incrementos
            current_joint_positions_[0] += left_x * velocity_factor;  // joint_base
            current_joint_positions_[1] += left_y * velocity_factor;  // joint_1
            current_joint_positions_[2] += right_y * velocity_factor; // joint_2 (stick Y)
            current_joint_positions_[3] += right_x * velocity_factor; // joint_3 (stick X)
            current_joint_positions_[4] += joint_4_input * velocity_factor; // joint_4
            current_joint_positions_[5] += gripper_input * velocity_factor; // right_gripper_joint
            // Aplicar límites de seguridad
            apply_joint_limits();
            // Enviar comandos separados para brazo y gripper
            send_arm_trajectory_command();
            send_gripper_trajectory_command();
            // Log de debug con valores actualizados (menos frecuente para suavidad visual)
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "🎮 Input: LX=%.2f LY=%.2f RX=%.2f RY=%.2f J4=%.2f G=%.2f → Pos: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                left_x, left_y, right_x, right_y, joint_4_input, gripper_input,
                current_joint_positions_[0], current_joint_positions_[1], current_joint_positions_[2], 
                current_joint_positions_[3], current_joint_positions_[4], current_joint_positions_[5]);
        }
    }
    
    void apply_joint_limits()
    {
        // Límites de seguridad para cada joint (en radianes)
        const std::array<double, 6> min_limits = {0, 0.4, 0.0, 0.0, 0.0, 0.1};
        const std::array<double, 6> max_limits = {3.14, 2.7, 3.14, 3.14, 3.14, 1.0};
        for (size_t i = 0; i < 6; ++i) {
            current_joint_positions_[i] = std::clamp(
                current_joint_positions_[i], min_limits[i], max_limits[i]);
        }
    }
    
    void send_arm_trajectory_command()
    {
        // Trajectory for arm (first 5 joints)
        auto traj = trajectory_msgs::msg::JointTrajectory();
        traj.joint_names = {
            "joint_base",
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4"
        };
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = {
            current_joint_positions_[0],
            current_joint_positions_[1],
            current_joint_positions_[2],
            current_joint_positions_[3],
            current_joint_positions_[4]
        };
        point.time_from_start = builtin_interfaces::msg::Duration();
        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 50000000;
        traj.points.push_back(point);
        trajectory_publisher_->publish(traj);
        RCLCPP_DEBUG(this->get_logger(), "📤 Trayectoria brazo enviada");
    }

    void send_gripper_trajectory_command()
    {
        // Trajectory for gripper (right_gripper_joint)
        auto traj = trajectory_msgs::msg::JointTrajectory();
        traj.joint_names = { "right_gripper_joint" };
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = { current_joint_positions_[5] };
        point.time_from_start = builtin_interfaces::msg::Duration();
        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 20000000;
        traj.points.push_back(point);
        if (!gripper_trajectory_publisher_) {
            gripper_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/gripper_controller/joint_trajectory", 10);
        }
        gripper_trajectory_publisher_->publish(traj);
        RCLCPP_DEBUG(this->get_logger(), "📤 Trayectoria gripper enviada");
    }

    // Variables miembro
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_trajectory_publisher_;
    std::array<double, 6> current_joint_positions_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GamepadTeleopSimple>();
    
    RCLCPP_INFO(node->get_logger(), "🚀 Iniciando teleoperación simple con gamepad...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
