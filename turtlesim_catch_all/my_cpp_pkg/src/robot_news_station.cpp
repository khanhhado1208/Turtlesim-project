
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class robot_news_station : public rclcpp::Node
{
    public:
        robot_news_station() : Node("robot_news_station"), robot_name_("R2D2")
        {
            publisher_ = this->create_publisher<std_msgs::msg::String>("robot_news", 10);
            timer_ = this->create_wall_timer(500ms, std::bind(&robot_news_station::PublishNews, this));
            RCLCPP_INFO(this->get_logger(), "Robot News Station has been started");
        }
    private:
        void PublishNews()
        {
            auto message = std_msgs::msg::String();
            message.data = "Hello, this is " + robot_name_ + " from the robot news station";
            publisher_->publish(message);
        }
        std::string robot_name_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_news_station>());
    rclcpp::shutdown();
    return 0;
}