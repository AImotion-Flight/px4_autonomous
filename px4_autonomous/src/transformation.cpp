#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

using namespace std::chrono_literals;

class Transformation : public rclcpp::Node {
public:
  Transformation() : Node("transformation") {
    this->uav_name = this->declare_parameter("uav_name", "uav");
    this->vehicle_local_position_topic = this->declare_parameter("vehicle_local_position", "/fmu/out/vehicle_local_position");
    
    rclcpp::QoS qos_sub(10);
    qos_sub.best_effort();

    this->tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    this->vehicle_local_position_sub =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        this->vehicle_local_position_topic,
        qos_sub,
        [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
          geometry_msgs::msg::TransformStamped t;

          t.header.stamp = this->get_clock()->now();
          t.header.frame_id = "world";
          t.child_frame_id = this->uav_name.c_str();

          t.transform.translation.x = msg->y;
          t.transform.translation.y = msg->x;
          t.transform.translation.z = -msg->z;

          this->tf_broadcaster->sendTransform(t);
          }
      );
  }

private:
  std::string uav_name;
  std::string vehicle_local_position_topic;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Transformation>());
  rclcpp::shutdown();
  return 0;
}
