#include <rclcpp/rclcpp.hpp>

#include <tim/msg/traveler_information_message.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <random>
#include <tuple>
#include <vector>
#include <string>

#include "tim.hpp"
#include "object.hpp"

using namespace std::chrono_literals;

// Infrastructure
class Infra: public rclcpp::Node
{
public:
	Infra(int id): Node("tim_infra")
	{
		id_ = id;

		// Initialize observation
		for (int i=0; i<20; ++i) {
			auto object = Object(i);
			observation_.push_back(object);
		}
		
		// Initialize ROS threads
		rsu = this->create_publisher<tim::msg::TravelerInformationMessage>(
			"tim",
			10
		);
		markers_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
			"infra_markers",
			10
		);
		timer = this->create_wall_timer(
			1000ms*dt_,
			std::bind(&Infra::timer_callback, this)
		);
	}

private:
	void timer_callback()
	{
		// Simulate objects
        for (Object & object: observation_) {
			object.update(dt_);
		}

		// Broadcast TIM
		TIM tim;
		write(tim, observation_);
			// remove undetected objects
		auto tim_rosmsg = tim.to_rosmsg();
		rsu->publish(tim_rosmsg);

		// Visualize TIM
		auto tim_rviz = tim.to_rviz();
		markers_publisher->publish(tim_rviz);
		
		// auto visualization = visualization_msgs::msg::MarkerArray(); {
		// 	auto edge_marker = extract_edge_marker(tim.regionals[0]);
		// 	visualization.markers.push_back(edge_marker);

		// 	for (auto & object: tim.regionals[0].objects) {
		// 		auto object_marker = extract_object_marker(tim.regionals[0].edge, object);
		// 		visualization.markers.push_back(object_marker);
		// 	}
		// }
		// markers_publisher->publish(visualization);

		// if (count_%10 == 0) {
		// 	RCLCPP_INFO(this->get_logger(), "TIM %d published", count_);
		// }
	}
	
	void write(TIM & tim, const std::vector<Object> & observation)
	{
		auto time = this->get_clock()->now();

		// 1. Message count
		tim.msgCnt = count_++;

		// 2. Data frame
		tim.data_frames[0].start_time = uint32_t(time.seconds()/60)%60;		// [minutes]
		tim.data_frames[0].duration_time = uint16_t(1);						// [minutes]

		// 3. Regional
		// 3.1. Edge
		tim.regionals[0].time_stamp = uint16_t(time.nanoseconds()/60)%60;	// [milliseconds]
		tim.regionals[0].processing_time = uint16_t(1);						// [milliseconds]
		tim.regionals[0].edge.id = uint32_t(id_);
		tim.regionals[0].edge.coordinate_system = uint16_t(5186);
		tim.regionals[0].edge.x = _Float64(pos_x_);
		tim.regionals[0].edge.y = _Float64(pos_y_);

		// 3.2. Objects
		tim.regionals[0].num_objects = uint8_t(observation.size());

		for (Object & object: observation_) {
			float x, y, Y;
			std::vector< std::tuple<float, float> > footprint;
			std::tie(x, y, Y, footprint) = object.get_states();
			float dx = x - pos_x_;
			float dy = y - pos_y_;
			float dist = sqrt(pow(dx, 2) + pow(dy, 2));

			Tim::Object tim_object; {

				// 3.2.1. State
				tim_object.id = uint32_t(object.get_id());
				tim_object.pose.x = int16_t(dx*100.);					// [m --> cm]
				tim_object.pose.y = int16_t(dy*100.);					// [m --> cm]
				tim_object.pose.angle = uint16_t(Y*180./M_PI/0.0125);	// [rad --> 0.0125deg]
				tim_object.velocity.x = int16_t(0./0.02);				// [m --> 0.02m]
				tim_object.velocity.y = int16_t(0./0.02);				// [m --> 0.02m]

				tim_object.footprint.num_nodes = uint8_t(footprint.size());
				for (std::tuple<float, float> & node: footprint) {
					Tim::Node tim_node;					
					tim_node.x = int16_t((std::get<0>(node) - pos_x_)*100.);	// [m --> cm]
					tim_node.y = int16_t((std::get<1>(node) - pos_y_)*100.);	// [m --> cm]							
					tim_object.footprint.nodes.push_back(tim_node);
				}

				// 3.2.2. Classification
				tim_object.object = uint8_t(	// OBJECT_CAR for detected, OBJECT_UNKNOWN for undetected
					dist < detection_range_ ? Tim::OBJECT::CAR : Tim::OBJECT::UNKNOWN);
				tim_object.object_classification_score = uint8_t(90);
				tim_object.location = uint8_t(Tim::LOCATION::CARLANE);
				tim_object.location_classification_score = uint8_t(90);
				tim_object.num_actions = uint8_t(1);
				Tim::Action tim_action; {
					tim_action.action = uint8_t(Tim::ACTION::MOVING);
					tim_action.action_classification_score = uint8_t(90);
				}
				tim_object.actions.push_back(tim_action);

				// 3.2.3. Trajectory forecasting
				tim_object.trajectory_forecasting.prediction_horizon = uint16_t(0);	// [milliseconds]
				tim_object.trajectory_forecasting.sampling_period = uint16_t(0);	// [milliseconds]
				tim_object.trajectory_forecasting.num_predictions = uint8_t(1);
				Tim::Prediction tim_prediction; {				// First prediction for current pose
					tim_prediction.num_nodes = uint8_t(1);
					Tim::Node tim_node; {
						tim_node.x = int16_t(tim_object.pose.x);	// [cm]
						tim_node.y = int16_t(tim_object.pose.y);	// [cm]
					}
					tim_prediction.nodes.push_back(tim_node);
				}
				tim_object.trajectory_forecasting.predictions.push_back(tim_prediction);
				tim_object.trajectory_forecasting_score = uint8_t(90);
			}
			tim.regionals[0].objects.push_back(tim_object);
		}
	}

	visualization_msgs::msg::Marker extract_edge_marker(tim::msg::RegionalExtension & regional)
	{
		float edge_height = 5.;

		visualization_msgs::msg::Marker edge_marker; {
			edge_marker.header.set__frame_id( std::to_string(regional.edge.coordinate_system) );
			edge_marker.set__id(regional.edge.id);
			edge_marker.pose.position.set__x(regional.edge.x);	// [m]
			edge_marker.pose.position.set__y(regional.edge.y);	// [m]
			edge_marker.pose.position.set__z(edge_height/2.);	// offset
			
			edge_marker.set__ns("edge");
			edge_marker.set__type(visualization_msgs::msg::Marker::CYLINDER);
			edge_marker.set__action(visualization_msgs::msg::Marker::MODIFY);
			edge_marker.scale.set__x(0.4);
			edge_marker.scale.set__y(0.4);
			edge_marker.scale.set__z(edge_height);
			edge_marker.color.set__a(0.5);
			edge_marker.color.set__r(1.);
			edge_marker.color.set__g(0.);
			edge_marker.color.set__b(0.);
		}
		return edge_marker;
	}

	visualization_msgs::msg::Marker extract_object_marker(
		tim::msg::Edge & edge,
		tim::msg::Object & object)
	{
		visualization_msgs::msg::Marker object_marker;

		object_marker.header.set__frame_id( std::to_string(edge.coordinate_system) );
		object_marker.set__id(object.id);
		for (const auto & node: object.footprint.nodes) {
			geometry_msgs::msg::Point point;
			point.set__x( node.x/100. + edge.x );						// [cm --> m] & offset
			point.set__y( node.y/100. + edge.y );						// [cm --> m] & offset
			object_marker.points.push_back(point);
		} {
			geometry_msgs::msg::Point point;
			point.set__x( object.footprint.nodes[0].x/100. + edge.x );	// [cm --> m] & offset
			point.set__y( object.footprint.nodes[0].y/100. + edge.y );	// [cm --> m] & offset
			object_marker.points.push_back(point);
		}
		
		object_marker.set__ns("object");
		object_marker.set__type(visualization_msgs::msg::Marker::LINE_STRIP);
		object_marker.set__action(visualization_msgs::msg::Marker::MODIFY);
		object_marker.scale.set__x(0.1);
		if (object.object == tim::msg::Object::OBJECT_UNKNOWN) {		// BLUE for undetected objects
			object_marker.color.set__a(0.5);
			object_marker.color.set__r(0.);
			object_marker.color.set__g(0.);
			object_marker.color.set__b(1.);
		}
		else {															// RED for detected objects
			object_marker.color.set__a(0.5);
			object_marker.color.set__r(1.);
			object_marker.color.set__g(0.);
			object_marker.color.set__b(0.);
		}			

		return object_marker;
	}

	rclcpp::Publisher<tim::msg::TravelerInformationMessage>::SharedPtr rsu;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher;
	rclcpp::TimerBase::SharedPtr timer;

	int id_;
	float pos_x_{ 10. };			// [m]
	float pos_y_{ -10. };			// [m]
	float detection_range_{ 30. };	// [m]
	float dt_{ 0.1 };				// [seconds]
	int count_{ 0 };
	std::vector<Object> observation_;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Infra>(0));
	rclcpp::shutdown();
	return 0;
}
