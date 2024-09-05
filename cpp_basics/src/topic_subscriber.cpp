#include "rclcpp/rclcpp.hpp"
#include "wego_msgs/msg/counter.hpp"

#include <memory>
#include <functional>
#include <iostream>

class TopicSubscriber : public rclcpp::Node
{
public:
    TopicSubscriber()
    : Node("topic_subscriber")
    {
        subscription_ = this->create_subscription<wego_msgs::msg::Counter>(
            "counter", 
            10,
            [](const wego_msgs::msg::Counter msg){
              std::cout << msg.count << std::endl;
            }
        );
    }

private:
    rclcpp::Subscription<wego_msgs::msg::Counter>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopicSubscriber>());
    rclcpp::shutdown();
    return 0;
}