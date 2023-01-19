#include <memory>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TrajectoryPublisher : public rclcpp::Node {
public:
  TrajectoryPublisher() : Node("trajectory_publisher") {
    this->pub = this->create_publisher<std_msgs::msg::String>("test", 10);
    this->timer = this->create_wall_timer(500ms, std::bind(&TrajectoryPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello";
    this->pub->publish(message);
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
