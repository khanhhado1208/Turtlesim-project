#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SmartphoneNode : public rclcpp::Node
{
public:
    SmartphoneNode() : Node("smartphone")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "robot_news", 10, std::bind(&SmartphoneNode::callbackRobotNews, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Smartphone has been started.");
    }

private:
    void callbackRobotNews(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Receiving '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_; 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                         // Initialize the ROS 2 system
    auto node = std::make_shared<SmartphoneNode>();   // Create an instance of SmartphoneNode
    rclcpp::spin(node);                               // Keep the node running
    rclcpp::shutdown();                               // Shutdown ROS 2
    return 0;
}
