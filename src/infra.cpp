#include <rclcpp/rclcpp.hpp>

#include <tim/msg/traveler_information_message.hpp>
#include <std_msgs/msg/string.hpp>
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
	Infra(int num_objects): Node("tim_infra")
	{
		// Initialize observation
		for (int i=0; i<num_objects; ++i) {
			auto object = Object(i);
			observation_.push_back(object);
		}
		
		// Initialize ROS threads
		rsu_rosmsg = this->create_publisher<tim::msg::TravelerInformationMessage>(
			"tim_rosmsg",
			10
		);
		rsu_string = this->create_publisher<std_msgs::msg::String>(
			"tim_string",
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
		TIM tim = perceive(observation_);

		// Broadcast TIM
		tim::msg::TravelerInformationMessage tim_rosmsg = tim.to_rosmsg();
		rsu_rosmsg->publish(tim_rosmsg);

		std::string s = tim.to_string();
		std_msgs::msg::String tim_string; {
			tim_string.set__data(s);
		}
		rsu_string->publish(tim_string);

		// Visualize TIM
		visualization_msgs::msg::MarkerArray tim_rviz = tim.to_rviz();
		markers_publisher->publish(tim_rviz);

		if (count_%10 == 0) {
			RCLCPP_INFO(this->get_logger(), "TIM %d published", count_);
		}
	}
	
	TIM perceive(const std::vector<Object> & observation)
	{
		TIM tim; {
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
						Tim::Node tim_node; {
							tim_node.x = int16_t((std::get<0>(node) - pos_x_)*100.);	// [m --> cm]
							tim_node.y = int16_t((std::get<1>(node) - pos_y_)*100.);	// [m --> cm]
						}
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
					Tim::Prediction tim_prediction; {					// First prediction for current pose
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
		return tim;
	}

	rclcpp::Publisher<tim::msg::TravelerInformationMessage>::SharedPtr rsu_rosmsg;	// Road-Side Unit (RSU)
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rsu_string;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher;
	rclcpp::TimerBase::SharedPtr timer;

	int id_{ 0 };
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
	rclcpp::spin(std::make_shared<Infra>(20));
	rclcpp::shutdown();
	return 0;
}
