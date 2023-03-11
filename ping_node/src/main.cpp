#include <rclcpp/rclcpp.hpp>
#include "rogilink2_interfaces/msg/frame.hpp"

class PingNode : public rclcpp::Node {
 public:
  PingNode() : Node("ping_node") {
    publisher_ =
        this->create_publisher<rogilink2_interfaces::msg::Frame>("rogilink2/send", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&PingNode::timer_callback, this));
  }

 private:
  void timer_callback() {
    auto message = rogilink2_interfaces::msg::Frame();
    message.name = "broadcast";
    message.hard_id = 0x3F;
    message.cmd_id = 0x01;
    message.is_remote = true;
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "ping");
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rogilink2_interfaces::msg::Frame>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PingNode>());
  rclcpp::shutdown();
  return 0;
}
