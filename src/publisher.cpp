#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class TimPublisher: public rclcpp::Node
{
public:
	TimPublisher(): Node("tim_publisher")
	{
		publisher = this->create_publisher<std_msgs::msg::String>(
			"tim",
			10
		);
		timer = this->create_wall_timer(
			500ms,
			std::bind(&TimPublisher::timer_callback, this)
		);
	}

private:
	void timer_callback()
	{
		auto message = std_msgs::msg::String();
		message.data = "TIM";
		publisher->publish(message);

		RCLCPP_INFO(this->get_logger(), "Publish TIM %d", count);
		count += 1;
	}

	rclcpp::TimerBase::SharedPtr timer;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

	unsigned int count{0};
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TimPublisher>());
	rclcpp::shutdown();
	return 0;
}
