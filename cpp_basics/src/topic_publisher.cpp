#include "rclcpp/rclcpp.hpp"
#include "wego_msgs/msg/counter.hpp"

#include <chrono>
#include <memory>
#include <functional>

using namespace std::chrono_literals;

class TopicPublisher : public rclcpp::Node
{
public:
    TopicPublisher()
    : Node("topic_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<wego_msgs::msg::Counter>("counter", 10);
        
        auto timer_callback = [this](){ 
            auto message = wego_msgs::msg::Counter();
            message.count = this->count_;
            this->publisher_->publish(message);
            this->count_++;
        }; // 람다 식 사용 -> C++ 11 부터 auto는 반환값으로 자동 형변환

        timer_ = this->create_wall_timer(
                500ms, 
                timer_callback
        );
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<wego_msgs::msg::Counter>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopicPublisher>());
    rclcpp::shutdown();
    return 0;
}
