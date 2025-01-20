#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoClientNode : public rclcpp::Node
{
public:
    AddTwoClientNode() : Node("add_two_ints_client")
    {
        RCLCPP_INFO(this->get_logger(), "Client node initialized. Making requests...");
        makeRequests();
    }

private:
    void makeRequests()
    {
        callAddTwoService(3, 8);
        callAddTwoService(4, 9);
    }

    void callAddTwoService(int a, int b)
    {
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Service not available, waiting again...");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        RCLCPP_INFO(this->get_logger(), "Sending request: a = %d, b = %d", a, b);

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Response received: %d + %d = %ld", a, b, response->sum);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoClientNode>();
    node.reset(); 
    rclcpp::shutdown();
    return 0;
}
