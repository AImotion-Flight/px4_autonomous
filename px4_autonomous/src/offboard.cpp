#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_autonomous_interfaces/action/execute_path.hpp"
#include "px4_autonomous_interfaces/msg/trajectory.hpp"

using namespace std::chrono_literals;

class Offboard : public rclcpp::Node {
public:
  Offboard() : Node("offboard"), armed(0) {
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
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(this->trajectory_setpoint_topic, qos_pub);

    this->vehicle_status_sub =
      this->create_subscription<px4_msgs::msg::VehicleStatus>(this->vehicle_status_topic,
							      qos_sub,
							      [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
								this->armed = msg->arming_state;
								this->mode = msg->nav_state;
							      });

    this->execute_path_action_server =
      rclcpp_action::create_server<px4_autonomous_interfaces::action::ExecutePath>(this,
										   "execute_path",
										   [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const px4_autonomous_interfaces::action::ExecutePath::Goal> goal) {
										     RCLCPP_INFO(this->get_logger(), "Received a path to execute");
										     (void)uuid;
										     (void)goal;
										     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
										   },
										   [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<px4_autonomous_interfaces::action::ExecutePath>> goal_handle) {
										     RCLCPP_INFO(this->get_logger(), "Received a request to cancel path execution");
										     (void)goal_handle;
										     return rclcpp_action::CancelResponse::ACCEPT;
										   },
										   [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<px4_autonomous_interfaces::action::ExecutePath>> goal_handle) {
										     std::thread{[this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<px4_autonomous_interfaces::action::ExecutePath>> goal_handle) {
										       RCLCPP_INFO(this->get_logger(), "Executing path ...");

										       const auto goal = goal_handle->get_goal();
										       auto feedback = std::make_shared<px4_autonomous_interfaces::action::ExecutePath::Feedback>();
										       auto result = std::make_shared<px4_autonomous_interfaces::action::ExecutePath::Result>();

										       rclcpp::Rate loop_rate(1);
										       unsigned int size = goal->trajectory.setpoints.size();
										       for (unsigned int i = 0; i < size && rclcpp::ok(); ++i) {
											 this->setpoint = goal->trajectory.setpoints[i];
											 
											 feedback->progress = i / size;
											 goal_handle->publish_feedback(feedback);
											 
											 loop_rate.sleep();
										       }

										       if (rclcpp::ok()) {
											 result->final_reached = true;
											 goal_handle->succeed(result);
											 RCLCPP_INFO(this->get_logger(), "Path executed");
										       }
										       
										     }, goal_handle}.detach();
										   });

    this->setpoint.position[2] = 2.5;

    this->setpoint_timer = this->create_wall_timer(10ms, [this]() {
      int timestamp = int(this->get_clock()->now().nanoseconds() / 1000);
      
      px4_msgs::msg::OffboardControlMode control_mode;
      control_mode.timestamp = timestamp;
      control_mode.position = true;
      control_mode.velocity = false;
      control_mode.acceleration = true;
      control_mode.attitude = false;
      control_mode.body_rate = false;
      
      px4_msgs::msg::TrajectorySetpoint sp;
      sp.timestamp = timestamp;
      sp.position = {this->setpoint.position[1] + 0.5f, this->setpoint.position[0] + 0.5f, -this->setpoint.position[2]};
      sp.velocity = {this->setpoint.velocity[1], this->setpoint.velocity[0], -this->setpoint.velocity[2]};
      sp.acceleration = {this->setpoint.acceleration[1], this->setpoint.acceleration[0], -this->setpoint.acceleration[2]};
      sp.yaw = this->setpoint.yaw;

      this->offboard_control_mode_pub->publish(control_mode);
      this->trajectory_setpoint_pub->publish(sp);
      });
  }

private:
  uint8_t armed;
  uint8_t mode;
  px4_msgs::msg::TrajectorySetpoint setpoint;
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
