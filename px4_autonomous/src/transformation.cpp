#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

using namespace std::chrono_literals;

class Transformation : public rclcpp::Node {
public:
  Transformation() : Node("transformation"), uav_id(0), initial_x(0.0), initial_y(0.0) {
    this->uav_id = this->declare_parameter("uav_id", 1);
    this->initial_x = this->declare_parameter("initial_x", 0.0);
    this->initial_y = this->declare_parameter("initial_y", 0.0);
    this->vehicle_local_position_topic = this->declare_parameter("vehicle_local_position", "/fmu/out/vehicle_local_position");
    
    rclcpp::QoS qos_sub(10);
    qos_sub.best_effort();

    this->tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    this->vehicle_local_position_sub =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/px4_" + std::to_string(this->uav_id) + this->vehicle_local_position_topic,
        qos_sub,
        [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
          geometry_msgs::msg::TransformStamped t;

          t.header.stamp = this->get_clock()->now();
          t.header.frame_id = "px4_uav_" + std::to_string(this->uav_id);
          t.child_frame_id = "uav_" + std::to_string(this->uav_id);

          t.transform.translation.x = msg->x;
          t.transform.translation.y = msg->y;
          t.transform.translation.z = msg->z;
          t.transform.rotation.x = 0.7071068;
          t.transform.rotation.y = 0.7071068;
          t.transform.rotation.z = 0;
          t.transform.rotation.w = 0;

          this->tf_broadcaster->sendTransform(t);
        }
      );
  }

private:
  uint8_t uav_id;
  double initial_x;
  double initial_y;
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
