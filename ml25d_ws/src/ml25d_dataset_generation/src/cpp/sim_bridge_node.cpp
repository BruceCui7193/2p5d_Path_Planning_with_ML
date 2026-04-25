#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SimBridgeNode : public rclcpp::Node {
public:
  SimBridgeNode() : Node("sim_bridge_node") {
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/ml25d/sim_bridge/status", 10);
    timer_ = this->create_wall_timer(1000ms, [this]() {
      std_msgs::msg::String msg;
      msg.data = "sim_bridge_alive";
      status_pub_->publish(msg);
    });
    RCLCPP_INFO(this->get_logger(), "sim_bridge_node started");
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
