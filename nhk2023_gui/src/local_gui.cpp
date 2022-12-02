#include <rclcpp/rclcpp.hpp>

class LocalGui : public rclcpp::Node {
 public:
  LocalGui() : Node("local_gui") {
    RCLCPP_INFO(this->get_logger(), "Local GUI is ready.");
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalGui>());
  rclcpp::shutdown();
  return 0;
}