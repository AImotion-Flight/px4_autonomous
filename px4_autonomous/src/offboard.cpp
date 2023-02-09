#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_autonomous_interfaces/action/execute_trajectory.hpp"
#include "px4_autonomous_interfaces/msg/trajectory.hpp"

using namespace std::chrono_literals;

class Offboard : public rclcpp::Node {
public:
  Offboard() : Node("offboard"), offboard_setpoint_counter_(0), armed(0) {
    this->vehicle_status_topic = this->declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status");
    this->offboard_control_mode_topic = this->declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode");
    this->trajectory_setpoint_topic = this->declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
    this->vehicle_command_topic = this->declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command");
    
    rclcpp::QoS qos_pub(10);
    rclcpp::QoS qos_sub(10);
    qos_sub.best_effort();

    this->vehicle_status_sub =
      this->create_subscription<px4_msgs::msg::VehicleStatus>(this->vehicle_status_topic, qos_sub, [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
        this->armed = msg->arming_state;
	      this->mode = msg->nav_state;
      });
    
    this->offboard_control_mode_pub =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(this->offboard_control_mode_topic, qos_pub);

    this->trajectory_setpoint_pub =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(this->trajectory_setpoint_topic, qos_pub);

    this->vehicle_command_pub =
      this->create_publisher<px4_msgs::msg::VehicleCommand>(this->vehicle_command_topic, qos_pub);

    this->execute_path_action_server =
      rclcpp_action::create_server<px4_autonomous_interfaces::action::ExecuteTrajectory>(this,
										   "execute_trajectory",
										   [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const px4_autonomous_interfaces::action::ExecuteTrajectory::Goal> goal) {
                          RCLCPP_INFO(this->get_logger(), "Received a trajectory to execute");
										     (void)uuid;
										     (void)goal;
										     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                       },
										   [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<px4_autonomous_interfaces::action::ExecuteTrajectory>> goal_handle) {
										     RCLCPP_INFO(this->get_logger(), "Received a request to cancel trajectory execution");
										     (void)goal_handle;
										     return rclcpp_action::CancelResponse::ACCEPT;
										   },
										   [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<px4_autonomous_interfaces::action::ExecuteTrajectory>> goal_handle) {
										     std::thread{[this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<px4_autonomous_interfaces::action::ExecuteTrajectory>> goal_handle) {
										       RCLCPP_INFO(this->get_logger(), "Executing path ...");

										       const auto goal = goal_handle->get_goal();
										       auto feedback = std::make_shared<px4_autonomous_interfaces::action::ExecuteTrajectory::Feedback>();
										       auto result = std::make_shared<px4_autonomous_interfaces::action::ExecuteTrajectory::Result>();

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
    this->setpoint.yaw = M_PI / 2;

    this->setpoint_timer = this->create_wall_timer(10ms, [this]() {
      if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
    });
  }

private:
  void subscribe_vehicle_status(px4_msgs::msg::VehicleStatus::UniquePtr msg);
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
  void publish_trajectory_setpoint();
  void publish_offboard_control_mode();
  void arm();
  void disarm();

  uint64_t offboard_setpoint_counter_;
  uint8_t armed;
  uint8_t mode;
  px4_msgs::msg::TrajectorySetpoint setpoint;
  std::string uav_name;
  std::string vehicle_status_topic;
  std::string offboard_control_mode_topic;
  std::string trajectory_setpoint_topic;
  std::string vehicle_command_topic;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub;
  rclcpp_action::Server<px4_autonomous_interfaces::action::ExecuteTrajectory>::SharedPtr execute_path_action_server;
  rclcpp::TimerBase::SharedPtr setpoint_timer;
};

void Offboard::publish_vehicle_command(uint16_t command, float param1, float param2) {
  px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	this->vehicle_command_pub->publish(msg);
}

void Offboard::publish_trajectory_setpoint() {
  px4_msgs::msg::TrajectorySetpoint sp;
  sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  sp.position = {this->setpoint.position[1] + 0.5f, this->setpoint.position[0] + 0.5f, -this->setpoint.position[2]};
  sp.velocity = {this->setpoint.velocity[1], this->setpoint.velocity[0], -this->setpoint.velocity[2]};
  sp.acceleration = {this->setpoint.acceleration[1], this->setpoint.acceleration[0], -this->setpoint.acceleration[2]};
  sp.yaw = this->setpoint.yaw;

  this->trajectory_setpoint_pub->publish(sp);
}

void Offboard::publish_offboard_control_mode() {
  px4_msgs::msg::OffboardControlMode control_mode;
  control_mode.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  control_mode.position = true;
  control_mode.velocity = true;
  control_mode.acceleration = true;
  control_mode.attitude = false;
  control_mode.body_rate = false;

  this->offboard_control_mode_pub->publish(control_mode);
}

void Offboard::arm() {
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void Offboard::disarm() {
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Offboard>());
  rclcpp::shutdown();
  return 0;
}
