#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_autonomous_interfaces/action/execute_path.hpp"

using namespace std::chrono_literals;

class Offboard : public rclcpp::Node {
public:
  Offboard() : Node("offboard"), theta(0), armed(0) {
    this->vehicle_status_topic =
      this->declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status");
    this->offboard_control_mode_topic =
      this->declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode");
    this->trajectory_setpoint_topic =
      this->declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
    
    rclcpp::QoS qos_pub(10);
    rclcpp::QoS qos_sub(10);
    qos_sub.best_effort();

    this->vehicle_status_sub =
      this->create_subscription<px4_msgs::msg::VehicleStatus>(this->vehicle_status_topic,
							      qos_sub,
							      [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
								this->armed = msg->arming_state;
								this->mode = msg->nav_state;
							      });
    
    this->offboard_control_mode_pub =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(this->offboard_control_mode_topic, qos_pub);

    this->trajectory_setpoint_pub =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(this->trajectory_setpoint_topic, qos_pubb);

    this->vehicle_status_sub =
      this->create_subscription<px4_msgs::msg::VehicleStatus>(this->vehicle_status_topic,
							      qos_sub,
							      [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
								this->armed = msg->arming_state;
								this->mode = msg->nav_state;
							      });

    this->setpoint_timer = this->create_wall_timer(100ms, [this]() {
      px4_msgs::msg::OffboardControlMode control_mode;
      control_mode.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
      control_mode.position = true;
      control_mode.velocity = false;
      control_mode.acceleration = false;
      control_mode.attitude = false;
      control_mode.body_rate = false;
      this->offboard_control_mode_pub->publish(control_mode);

      px4_msgs::msg::TrajectorySetpoint setpoint;
      setpoint.timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
      setpoint.position = {2.0f * std::cos(this->theta),
			   2.0f * std::sin(this->theta), -5.0};
      this->trajectory_setpoint_pub->publish(setpoint);

      this->theta = this->theta + (M_PI / 50);
      if (this->theta > 2 * M_PI)
        this->theta = 0;
    });
  }

private:
  float theta;
  uint8_t armed;
  uint8_t mode;
  std::string uav_name;
  std::string vehicle_status_topic;
  std::string offboard_control_mode_topic;
  std::string trajectory_setpoint_topic;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub;
  rclcpp_action::Server<px4_autonomous_interfaces::action::ExecutePath>::SharedPtr execute_path_action_server;
  rclcpp::TimerBase::SharedPtr setpoint_timer;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Offboard>());
  rclcpp::shutdown();
  return 0;
}
