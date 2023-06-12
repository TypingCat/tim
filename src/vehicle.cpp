#include <rclcpp/rclcpp.hpp>

#include <tim/msg/traveler_information_message.hpp>
#include <visualization_msgs/msg/marker.hpp>

using std::placeholders::_1;

// Vehicle
class Vehicle: public rclcpp::Node
{
public:
	Vehicle(): Node("tim_vehicle")
	{
		initialize_objects();

		obu = this->create_subscription<tim::msg::TravelerInformationMessage>(
			"tim",
			10,
            std::bind(&Vehicle::tim_callback, this, _1)
		);
		marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>(
			"marker_vehicle",
			10
		);
	}

private:
	void initialize_objects()
    {
        //
    }

    void tim_callback(const tim::msg::TravelerInformationMessage & tim)
    {
        update_objects(tim);

        auto marker = generate_marker();

        marker_publisher->publish(marker);
    }

	void update_objects(tim::msg::TravelerInformationMessage)
    {
        //
    }

    visualization_msgs::msg::Marker generate_marker()
    {
        auto marker = visualization_msgs::msg::Marker();

        //

        return marker;
    }

	rclcpp::Subscription<tim::msg::TravelerInformationMessage>::SharedPtr obu;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
	rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Vehicle>());
	rclcpp::shutdown();
	return 0;
}
