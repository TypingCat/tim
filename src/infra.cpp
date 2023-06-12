#include <rclcpp/rclcpp.hpp>

#include <tim/msg/traveler_information_message.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>

using namespace std::chrono_literals;

// Infrastructure
class Infra: public rclcpp::Node
{
public:
	Infra(): Node("tim_infra")
	{
		initialize_objects();

		rsu = this->create_publisher<tim::msg::TravelerInformationMessage>(
			"tim",
			10
		);
		marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>(
			"marker_infra",
			10
		);
		timer = this->create_wall_timer(
			500ms,
			std::bind(&Infra::timer_callback, this)
		);
	}

private:
	void initialize_objects(){ }

	void timer_callback()
	{
        simulate_objects();

		auto tim = generate_tim();
		auto marker = generate_marker();

		rsu->publish(tim);
		marker_publisher->publish(marker);		
	}
	
	void simulate_objects()
	{
		//
	}

	tim::msg::TravelerInformationMessage generate_tim()
	{
		auto tim = tim::msg::TravelerInformationMessage();

		//

		return tim;
	}

	visualization_msgs::msg::Marker generate_marker()
	{
		auto marker = visualization_msgs::msg::Marker();

        //

        return marker;
	}

	rclcpp::Publisher<tim::msg::TravelerInformationMessage>::SharedPtr rsu;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
	rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Infra>());
	rclcpp::shutdown();
	return 0;
}
