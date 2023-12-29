#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
using namespace std::chrono_literals; // to use 1s

class SimplePublisher : public rclcpp::Node // inherit from Node class
{
public: 
    SimplePublisher() : Node("simple_publisher"), counter_(0) // constructor of base class
    {
        pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
        timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this)); // Callback defined in SimplePublisher class, use this one

        RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
    }

    void timerCallback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello ROS 2 - counter: " + std::to_string(counter_++);
        pub_->publish(message); // on pub_ object execute publish-function
    }

private:
    unsigned int counter_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePublisher>();
    rclcpp::spin(node); // run continuously
    rclcpp::shutdown(); // destructor
    return 0;
}