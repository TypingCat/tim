#include <rclcpp/rclcpp.hpp>

#include <tim/msg/traveler_information_message.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
		markers_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"markers_vehicle",
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

        auto markers = generate_marker();

        markers_publisher->publish(markers);
    }

	void update_objects(tim::msg::TravelerInformationMessage)
    {
        //
    }

    visualization_msgs::msg::MarkerArray generate_marker()
    {
        auto markers = visualization_msgs::msg::MarkerArray();

        //

        return markers;
    }

	rclcpp::Subscription<tim::msg::TravelerInformationMessage>::SharedPtr obu;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher;
	rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Vehicle>());
	rclcpp::shutdown();
	return 0;
}
