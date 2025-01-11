#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("cpp_test"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello, CPP Node");

        // Create a timer that triggers the timer_callback every second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyNode::timer_callback, this));
    }

private:
    
    void timer_callback()
    {
        counter_ += 1;
        RCLCPP_INFO(this->get_logger(), "Hello, CPP Timer %d", counter_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int counter_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                       // Initialize the ROS 2 system
    auto node = std::make_shared<MyNode>();         // Create an instance of MyNode
    rclcpp::spin(node);                             // Keep the node running
    rclcpp::shutdown();                             // Shutdown ROS 2
    return 0;
}
