#include <chrono>
#include <cmath>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_autonomous_interfaces/action/execute_trajectory.hpp"
#include "px4_autonomous_interfaces/msg/trajectory.hpp"

class Offboard : public rclcpp::Node {
public:
  Offboard() : Node("offboard"), executing_trajectory(false), offboard_setpoint_counter_(0), armed(0), mode(0), d_safe(1), uav_id(0), partner_id(0), setpoint_rate(0), system_id(0) {
    this->uav_id = this->declare_parameter("uav_id", 1);
    this->partner_id = this->declare_parameter("partner_id", 1);
    this->vehicle_status_topic = this->declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status");
    this->offboard_control_mode_topic = this->declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode");
    this->trajectory_setpoint_topic = this->declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
    this->vehicle_command_topic = this->declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command");
    this->vehicle_local_position_topic = this->declare_parameter("vehicle_local_position", "/fmu/out/vehicle_local_position");
    this->setpoint_rate = this->declare_parameter("setpoint_rate", 100);
    this->system_id = this->declare_parameter("system_id", 100);

    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer);
    
    rclcpp::QoS qos_pub(10);
    rclcpp::QoS qos_sub(10);
    qos_sub.best_effort();

    this->vehicle_status_sub =
      this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/px4_" + std::to_string(this->uav_id) + this->vehicle_status_topic,
        qos_sub,
        [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
          this->armed = msg->arming_state;
	        this->mode = msg->nav_state;
        }
      );

    if (this->uav_id != 1) {
      this->leader_vehicle_local_position_sub =
        this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          "/px4_1" + this->vehicle_local_position_topic,
          qos_sub,
          [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
            tf2::Vector3 point(msg->x, msg->y, msg->z);
            tf2::Vector3 transformed = this->transform_point(point, "px4_uav_1", "gazebo");
            this->leader_pos[0] = transformed.x();
            this->leader_pos[1] = transformed.y();
            this->leader_pos[2] = transformed.z();
          }
        );

      this->partner_vehicle_local_position_sub =
        this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          "/px4_" + std::to_string(this->partner_id) + this->vehicle_local_position_topic,
          qos_sub,
          [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
            tf2::Vector3 point(msg->x, msg->y, msg->z);
            tf2::Vector3 transformed = this->transform_point(point, "px4_uav_" + std::to_string(this->partner_id), "gazebo");
            this->partner_pos[0] = transformed.x();
            this->partner_pos[1] = transformed.y();
            this->partner_pos[2] = transformed.z();
          }
        );
    }

    this->vehicle_local_position_sub =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/px4_" + std::to_string(this->uav_id) + this->vehicle_local_position_topic,
        qos_sub,
        [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
          tf2::Vector3 point(msg->x, msg->y, msg->z);
          tf2::Vector3 transformed = this->transform_point(point, "px4_uav_" + std::to_string(this->uav_id), "gazebo");
          this->pos[0] = transformed.x();
          this->pos[1] = transformed.y();
          this->pos[2] = transformed.z();
        }
      );
    
    this->offboard_control_mode_pub =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>("/px4_" + std::to_string(this->uav_id) + this->offboard_control_mode_topic, qos_pub);

    this->trajectory_setpoint_pub =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/px4_" + std::to_string(this->uav_id) + this->trajectory_setpoint_topic, qos_pub);

    this->vehicle_command_pub =
      this->create_publisher<px4_msgs::msg::VehicleCommand>("/px4_" + std::to_string(this->uav_id) + this->vehicle_command_topic, qos_pub);

    this->execute_trajectory_action_server =
      rclcpp_action::create_server<px4_autonomous_interfaces::action::ExecuteTrajectory>(
        this,
        "execute_trajectory",
        [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const px4_autonomous_interfaces::action::ExecuteTrajectory::Goal> goal) {
          RCLCPP_INFO(this->get_logger(), "Received a trajectory to execute");
          (void)uuid;
          (void)goal;
          this->executing_trajectory = true;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<px4_autonomous_interfaces::action::ExecuteTrajectory>> goal_handle) {
          RCLCPP_INFO(this->get_logger(), "Received a request to cancel trajectory execution");
          (void)goal_handle;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<px4_autonomous_interfaces::action::ExecuteTrajectory>> goal_handle) {
              std::thread{[this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<px4_autonomous_interfaces::action::ExecuteTrajectory>> goal_handle) {
          RCLCPP_INFO(this->get_logger(), "Executing trajectory ...");

          const auto goal = goal_handle->get_goal();
          auto feedback = std::make_shared<px4_autonomous_interfaces::action::ExecuteTrajectory::Feedback>();
          auto result = std::make_shared<px4_autonomous_interfaces::action::ExecuteTrajectory::Result>();

          rclcpp::Rate loop_rate(1.0);
          unsigned int size = goal->trajectory.setpoints.size();
          for (unsigned int i = 0; i < size && rclcpp::ok(); ++i) {
            this->set_position_setpoint(goal->trajectory.setpoints[i].position[0] + 0.5, goal->trajectory.setpoints[i].position[1] + 0.5, goal->trajectory.setpoints[i].position[2] + 0.5, M_PI / 2);
            feedback->progress = float(i) / float(size);
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
          }

          if (rclcpp::ok()) {
            result->final_reached = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Trajectory executed");
            this->executing_trajectory = false;
          }
        }, goal_handle}.detach();
        }
      );

    this->control_timer = this->create_wall_timer(std::chrono::milliseconds(this->setpoint_rate), [this]() {
      if (this->executing_trajectory == true) {
        return;
      }

      float x = 0.5;
      float y = 0.5;
      float z = 1.5;

      if (this->uav_id == 2) {
        x = (this->leader_pos[0] - this->d_safe) + (this->partner_pos[0] - this->pos[0] + this->d_safe);
        y = (this->leader_pos[1] - this->d_safe) + (this->partner_pos[1] - this->pos[1] + this->d_safe);
        z = (this->leader_pos[2]);
      } else if (this->uav_id == 3) {
        x = (this->leader_pos[0] - this->d_safe) + (this->partner_pos[0] - this->pos[0] - this->d_safe);
        y = (this->leader_pos[1] - this->d_safe) + (this->partner_pos[1] - this->pos[1] - this->d_safe);
        z = (this->leader_pos[2]);
      }

      float yaw = M_PI / 2;
      this->set_position_setpoint(x, y, z, yaw);
    });

    this->setpoint_timer = this->create_wall_timer(std::chrono::milliseconds(this->setpoint_rate), [this]() {
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
  void set_position_setpoint(float x, float y, float z, float yaw);
  void publish_trajectory_setpoint();
  void publish_offboard_control_mode();
  void arm();
  void disarm();
  tf2::Vector3 transform_point(tf2::Vector3 point, std::string from, std::string to);

  bool executing_trajectory;
  uint64_t offboard_setpoint_counter_;
  uint8_t armed;
  uint8_t mode;
  double d_safe;
  px4_msgs::msg::TrajectorySetpoint setpoint;
  std::array<float, 3> leader_pos;
  std::array<float, 3> partner_pos;
  std::array<float, 3> pos;
  uint8_t uav_id;
  uint8_t partner_id;
  std::string vehicle_status_topic;
  std::string offboard_control_mode_topic;
  std::string trajectory_setpoint_topic;
  std::string vehicle_command_topic;
  std::string vehicle_local_position_topic;
  uint64_t setpoint_rate;
  uint8_t system_id;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr leader_vehicle_local_position_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr partner_vehicle_local_position_sub;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub;
  rclcpp_action::Server<px4_autonomous_interfaces::action::ExecuteTrajectory>::SharedPtr execute_trajectory_action_server;
  rclcpp::TimerBase::SharedPtr control_timer;
  rclcpp::TimerBase::SharedPtr setpoint_timer;
};

void Offboard::publish_vehicle_command(uint16_t command, float param1, float param2) {
  px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = this->uav_id + 1;
	msg.target_component = 1;
	msg.source_system = this->system_id;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	this->vehicle_command_pub->publish(msg);
}

void Offboard::set_position_setpoint(float x, float y, float z, float yaw) {
  tf2::Vector3 transformed = this->transform_point(tf2::Vector3(x, y, z), "gazebo", "px4_uav_" + std::to_string(this->uav_id));
  this->setpoint.position = {(float)transformed.x(), (float)transformed.y(), (float)transformed.z()};
  this->setpoint.yaw = yaw;
}

void Offboard::publish_trajectory_setpoint() {
  px4_msgs::msg::TrajectorySetpoint sp = this->setpoint;
  sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;

  this->trajectory_setpoint_pub->publish(sp);
}

void Offboard::publish_offboard_control_mode() {
  px4_msgs::msg::OffboardControlMode control_mode;
  control_mode.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  control_mode.position = true;
  control_mode.velocity = false;
  control_mode.acceleration = false;
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

tf2::Vector3 Offboard::transform_point(tf2::Vector3 point, std::string from, std::string to) {
  geometry_msgs::msg::TransformStamped t;
  try {
    t = this->tf_buffer->lookupTransform(to, from, tf2::TimePointZero);
  } catch (tf2::LookupException& ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", from.c_str(), to.c_str(), ex.what());
    return tf2::Vector3();
  }
  tf2::Vector3 translation(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
  tf2::Quaternion rotation(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
  tf2::Transform transform(rotation, translation);
  return transform * point;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Offboard>());
  rclcpp::shutdown();
  return 0;
}