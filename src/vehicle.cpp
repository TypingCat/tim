#include <rclcpp/rclcpp.hpp>

#include <tim/msg/traveler_information_message.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <random>
#include <string>

#include "tim.hpp"
#include "object.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// Vehicle
class Vehicle: public rclcpp::Node, Object
{
public:
	Vehicle(): Node("tim_vehicle"), Object()
	{
		// Get parameters
		this->declare_parameter<int>("id", 0);
		id_ = this->get_parameter("id").as_int();
		
		this->declare_parameter<std::string>("obu_mode", "string");
		std::string obu_mode = this->get_parameter("obu_mode").as_string();
		
		this->declare_parameter< std::vector<double> >("color", {0., 1., 0.});
		color_ = this->get_parameter("color").as_double_array();

		// Initialize ROS threads
		if (obu_mode == "rosmsg") {
			obu_rosmsg = this->create_subscription<tim::msg::TravelerInformationMessage>(
				"tim_rosmsg",
				10,
				std::bind(&Vehicle::tim_rosmsg_callback, this, _1)
			);
		}
		else if (obu_mode == "string") {
			obu_string = this->create_subscription<std_msgs::msg::String>(
				"tim_string",
				10,
				std::bind(&Vehicle::tim_string_callback, this, _1)
			);
		}
		markers_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"vehicle_markers",
			10
		);
		timer = this->create_wall_timer(
			100ms,
			std::bind(&Vehicle::timer_callback, this)
		);
	}

private:
	void timer_callback()
	{
		this->update(dt_);
		
		auto visualization = visualization_msgs::msg::MarkerArray(); {

			// Visualize ICAD vehicle			
			auto vehicle_marker = generate_vehicle_marker(this);
			visualization.markers.push_back(vehicle_marker);

			// Visualize TIM
			if (observation_.size() != 0) {
				Tim::Edge edge = observation_.back().regionals[0].edge;

				for (auto & object: observation_.back().regionals[0].objects) {
					auto perception_marker = generate_perception_marker(edge, object, this);
					visualization.markers.push_back(perception_marker);
				}

				observation_.clear();
			}
		}
		markers_publisher->publish(visualization);
	}

    void tim_rosmsg_callback(const tim::msg::TravelerInformationMessage & tim_rosmsg)
    {
		observation_.push_back(TIM(tim_rosmsg));
		
		if (observation_.back().msgCnt%10 == 0) {
			RCLCPP_INFO(this->get_logger(), "TIM(rosmsg) %d subscribed", observation_.back().msgCnt);
		}
    }

    void tim_string_callback(const std_msgs::msg::String & tim_string)
    {
		observation_.push_back(TIM(tim_string.data));
		
		if (observation_.back().msgCnt%10 == 0) {
			RCLCPP_INFO(this->get_logger(), "TIM(rosmsg) %d subscribed", observation_.back().msgCnt);
		}
    }

    visualization_msgs::msg::Marker generate_vehicle_marker(Vehicle * vehicle)
    {
		float x, y, Y;
		std::vector< std::tuple<float, float> > footprint;
		std::tie(x, y, Y, footprint) = vehicle->get_states();

        visualization_msgs::msg::Marker vehicle_marker; {
			vehicle_marker.header.set__frame_id( std::to_string(vehicle->coordinate_system_) );
			vehicle_marker.set__id(vehicle->id_);
			for (std::tuple<float, float> & node: footprint) {
				geometry_msgs::msg::Point point;
				point.set__x(std::get<0>(node));
				point.set__y(std::get<1>(node));
				vehicle_marker.points.push_back(point);
			}
			geometry_msgs::msg::Point point;
			point.set__x(vehicle_marker.points[0].x);
			point.set__y(vehicle_marker.points[0].y);
			vehicle_marker.points.push_back(point);
			
			vehicle_marker.set__ns("vehicle");
			vehicle_marker.set__type(visualization_msgs::msg::Marker::LINE_STRIP);
			vehicle_marker.set__action(visualization_msgs::msg::Marker::MODIFY);
			vehicle_marker.scale.set__x(0.1);
			vehicle_marker.color.set__a(0.5);
			vehicle_marker.color.set__r(color_[0]);
			vehicle_marker.color.set__g(color_[1]);
			vehicle_marker.color.set__b(color_[2]);
			vehicle_marker.lifetime.set__sec(0);
			vehicle_marker.lifetime.set__nanosec(100000000);
		}
        return vehicle_marker;
    }

	visualization_msgs::msg::Marker generate_perception_marker(Tim::Edge & edge, Tim::Object & object, Vehicle * vehicle)
    {
		float x, y, Y;
		std::vector< std::tuple<float, float> > footprint;
		std::tie(x, y, Y, footprint) = vehicle->get_states();

        visualization_msgs::msg::Marker perception_marker; {
			perception_marker.header.set__frame_id( std::to_string(vehicle->coordinate_system_) );
			perception_marker.set__id(object.id);
			geometry_msgs::msg::Point vehicle_point; {
				vehicle_point.set__x(x);								// [m]
				vehicle_point.set__y(y);								// [m]
			}
			perception_marker.points.push_back(vehicle_point);
			geometry_msgs::msg::Point object_point; {
				object_point.set__x( object.pose.x/100. + edge.x );		// [cm --> m] & offset
				object_point.set__y( object.pose.y/100. + edge.y );		// [cm --> m] & offset
			}
			perception_marker.points.push_back(object_point);

			perception_marker.set__ns( "perception_" + std::to_string(vehicle->id_) );
			perception_marker.set__type(visualization_msgs::msg::Marker::LINE_LIST);
			perception_marker.set__action(visualization_msgs::msg::Marker::MODIFY);
			perception_marker.scale.set__x(0.1);
			perception_marker.color.set__a(0.2);
			perception_marker.color.set__r(color_[0]);
			perception_marker.color.set__g(color_[1]);
			perception_marker.color.set__b(color_[2]);
			perception_marker.lifetime.set__sec(0);
			perception_marker.lifetime.set__nanosec(100000000);
		}
        return perception_marker;
    }

	rclcpp::Subscription<tim::msg::TravelerInformationMessage>::SharedPtr obu_rosmsg;	// On-Board Unit (OBU)
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr obu_string;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher;
	rclcpp::TimerBase::SharedPtr timer;

	int id_;
	std::vector<double> color_;

	float dt_{ 0.1 };					// [seconds]
	int coordinate_system_{ 5186 };		// EPSG
	std::vector<TIM> observation_;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Vehicle>());
	rclcpp::shutdown();
	return 0;
}
