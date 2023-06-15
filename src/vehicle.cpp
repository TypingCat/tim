#include <rclcpp/rclcpp.hpp>

#include <tim/msg/traveler_information_message.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <random>
#include <string>

#include "object.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// Vehicle
class Vehicle: public rclcpp::Node, Object
{
public:
	Vehicle(int id): Node("tim_vehicle"), Object(id)
	{
		id_ = id;

		// Initialize ROS threads
		obu = this->create_subscription<tim::msg::TravelerInformationMessage>(
			"tim",
			10,
            std::bind(&Vehicle::tim_callback, this, _1)
		);
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
		// Simulate this vehicle
		this->update(dt_);

		// Visualization
		auto visualization = visualization_msgs::msg::MarkerArray(); {
			auto vehicle_marker = extract_vehicle_marker(this);
			visualization.markers.push_back(vehicle_marker);

			if (observation_.size() != 0) {
				auto edge = observation_.back().regionals[0].edge;

				for (auto & object: observation_.back().regionals[0].objects) {
					auto perception_marker = extract_perception_marker(edge, object, this);
					visualization.markers.push_back(perception_marker);
				}

				observation_.clear();
			}
		}
		markers_publisher->publish(visualization);
	}

    void tim_callback(const tim::msg::TravelerInformationMessage & tim)
    {
		observation_.push_back(tim);
		
		if (tim.msg_cnt%10 == 0) {
			RCLCPP_INFO(this->get_logger(), "TIM %d subscribed", tim.msg_cnt);
		}
    }

    visualization_msgs::msg::Marker extract_vehicle_marker(Vehicle * vehicle)
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
			vehicle_marker.color.set__r(0.);
			vehicle_marker.color.set__g(1.);	// GREEN for vehicles
			vehicle_marker.color.set__b(0.);
		}
        return vehicle_marker;
    }

	visualization_msgs::msg::Marker extract_perception_marker(
		tim::msg::Edge & edge,
		tim::msg::Object & object,
		Vehicle * vehicle)
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
			perception_marker.color.set__r(0.);
			perception_marker.color.set__g(1.);	// GREEN for vehicles
			perception_marker.color.set__b(0.);
		}
        return perception_marker;
    }

	rclcpp::Subscription<tim::msg::TravelerInformationMessage>::SharedPtr obu;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher;
	rclcpp::TimerBase::SharedPtr timer;

	int id_;
	float dt_{ 0.1 };					// [seconds]
	int coordinate_system_{ 5186 };		// EPSG
	std::vector<tim::msg::TravelerInformationMessage> observation_;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Vehicle>(0));
	rclcpp::shutdown();
	return 0;
}
