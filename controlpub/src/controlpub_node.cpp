#include <string>

#include "byd_custom_msgs/msg/control_res.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class ControlPub : public rclcpp::Node
{
public:
  ControlPub()
  : Node("controlpub")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/cmd_vel");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/control_to_uart");

    publisher_ = create_publisher<byd_custom_msgs::msg::ControlRes>(output_topic, 10);
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        byd_custom_msgs::msg::ControlRes control_msg;
        control_msg.v = msg->linear.x;
        control_msg.w = msg->angular.z;
        control_msg.v_lift = 0.0;
        control_msg.w_rotation = 0.0;
        publisher_->publish(control_msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "Forwarding %s to %s as byd_custom_msgs/msg/ControlRes",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<byd_custom_msgs::msg::ControlRes>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlPub>());
  rclcpp::shutdown();
  return 0;
}
