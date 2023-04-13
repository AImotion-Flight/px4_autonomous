#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "nav_msgs/msg/path.hpp"

class Visualization : public rclcpp::Node {
public:
  Visualization() : Node("visualization"), uav_id(0), initial_x(0), initial_y(0) {
    this->uav_id = this->declare_parameter("uav_id", 1);
    this->initial_x = this->declare_parameter("initial_x", 0);
    this->initial_y = this->declare_parameter("initial_y", 0);
    this->vehicle_attitude_topic = this->declare_parameter("vehicle_attitude", "/fmu/out/vehicle_attitude");
    this->vehicle_local_position_topic = this->declare_parameter("vehicle_local_position", "/fmu/out/vehicle_local_position");
    this->path_topic = this->declare_parameter("path_topic", "visualization/path");

    rclcpp::QoS qos_pub(10);
    rclcpp::QoS qos_sub(10);
    qos_sub.best_effort();

    this->vehicle_local_position_sub =
        this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          "/px4_" + std::to_string(this->uav_id) + this->vehicle_local_position_topic,
          qos_sub,
          [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
            this->position[0] = msg->x;
            this->position[1] = msg->y;
            this->position[2] = msg->z;
          }
        );

    this->vehicle_attitude_sub =
        this->create_subscription<px4_msgs::msg::VehicleAttitude>(
          "/px4_" + std::to_string(this->uav_id) + this->vehicle_attitude_topic,
          qos_sub,
          [this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
            this->attitude[0] = msg->q[0];
            this->attitude[1] = msg->q[1];
            this->attitude[2] = msg->q[2];
            this->attitude[3] = msg->q[3];
          }
        );

    this->path_pub =
      this->create_publisher<nav_msgs::msg::Path>(this->path_topic, qos_pub);

    this->path_timer = this->create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "px4_uav_" + std::to_string(this->uav_id);
        pose.pose.position.x = this->position[0];
        pose.pose.position.y = this->position[1];
        pose.pose.position.z = this->position[2];
        pose.pose.orientation.w = this->attitude[0];
        pose.pose.orientation.x = this->attitude[1];
        pose.pose.orientation.y = this->attitude[2];
        pose.pose.orientation.z = this->attitude[3];
        this->path_poses.push_back(pose);
        
        nav_msgs::msg::Path path;
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "px4_uav_" + std::to_string(this->uav_id);
        path.poses = this->path_poses;
        this->path_pub->publish(path);
      }
    );
  }

private:
  float position[3];
  float attitude[4];
  std::vector<geometry_msgs::msg::PoseStamped> path_poses;
  uint8_t uav_id;
  double initial_x;
  double initial_y;
  std::string vehicle_attitude_topic;
  std::string vehicle_local_position_topic;
  std::string path_topic;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::TimerBase::SharedPtr path_timer;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Visualization>());
  rclcpp::shutdown();
  return 0;
}
