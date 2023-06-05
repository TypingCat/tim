#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class TimSubscriber: public rclcpp::Node
{
public:
    TimSubscriber(): Node("tim_subscriber")
    {
        subscription = this->create_subscription<std_msgs::msg::String>(
            "tim",
            10,
            std::bind(&TimSubscriber::topic_callback, this, _1)
        );
    }

private:
    void topic_callback(const std_msgs::msg::String & message) const
    {
        RCLCPP_INFO(this->get_logger(), "Subscribe %s", message.data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TimSubscriber>());
	rclcpp::shutdown();
	return 0;
}