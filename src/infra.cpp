#include <rclcpp/rclcpp.hpp>

#include <tim/msg/traveler_information_message.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <random>
#include <tuple>
#include <vector>
#include <string>

using namespace std::chrono_literals;

// Object with uniform orbit
class Object
{
public:
	Object(int id)
	{
		id_ = id;

		// Initialize random engine
		std::random_device rd;
		std::mt19937_64 gen(rd());
		std::normal_distribution<double> dist(0, 1);

		// Initialize object states
		footprint_radius_ = 2.*dist(gen) + 1.;
		footprint_shape_ = M_PI/32.*dist(gen) + M_PI/8.;
		
		orbital_center_x_ = 10.*dist(gen) - 5.;
		orbital_center_y_ = 10.*dist(gen) - 5.;
		orbital_radius_ = 20.*dist(gen) - 10.;
		orbital_position_ = 6.28*dist(gen);
		orbital_velocity_ = 0.05*dist(gen) + 0.1;
	}

	void update(float dt)
	{
		orbital_position_ += orbital_velocity_*dt;
		orbital_position_ = fmod(orbital_position_, 2*M_PI);
	}

	int get_id() { return id_; }

	std::tuple<float, float, float, std::vector< std::tuple<float, float> > > get_states()
	{
		// Pose
		float x = orbital_center_x_ + orbital_radius_*cos(orbital_position_);
		float y = orbital_center_y_ + orbital_radius_*sin(orbital_position_);
		float Y = fmod(orbital_position_ + (orbital_velocity_ > 0 ? 1. : -1.)*M_PI_2, 2*M_PI);

		// Footprint
		std::vector< std::tuple<float, float> > footprint;
		footprint.push_back(std::make_tuple(
			x + footprint_radius_*cos(Y + footprint_shape_),
			y + footprint_radius_*sin(Y + footprint_shape_)
		));
		footprint.push_back(std::make_tuple(
			x + footprint_radius_*cos(Y - footprint_shape_),
			y + footprint_radius_*sin(Y - footprint_shape_)
		));
		footprint.push_back(std::make_tuple(
			x + footprint_radius_*cos(Y + footprint_shape_ + M_PI),
			y + footprint_radius_*sin(Y + footprint_shape_ + M_PI)
		));
		footprint.push_back(std::make_tuple(
			x + footprint_radius_*cos(Y - footprint_shape_ - M_PI),
			y + footprint_radius_*sin(Y - footprint_shape_ - M_PI)
		));

		return std::make_tuple(x, y, Y, footprint);
	}

private:
	int id_;

	float footprint_radius_;
	float footprint_shape_;

	float orbital_center_x_;
    float orbital_center_y_;
    float orbital_radius_;
    float orbital_position_;
    float orbital_velocity_;	
};

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
			"markers_infra",
			10
		);
		timer = this->create_wall_timer(
			100ms,
			std::bind(&Infra::timer_callback, this)
		);
	}

private:
	void timer_callback()
	{
		// Simulate objects
        for (Object & object: observation_) {
			object.update(0.5);
		}

		// Generate messages
		auto tim = generate_tim(observation_);
		auto tim_visualization = generate_markers(tim);

		// Publish messages
		rsu->publish(tim);
		markers_publisher->publish(tim_visualization);

		if (count_%10 == 0) {
			RCLCPP_INFO(this->get_logger(), "%d TIM published", count_);
		}
	}

	// Reference:
	// Communication Protocal Design Specification for Connected Autonomous Driving Service Edge AI v0.5
	tim::msg::TravelerInformationMessage generate_tim(const std::vector<Object> & observation)
	{
		auto tim = tim::msg::TravelerInformationMessage(); {
			auto time = this->get_clock()->now();

			// 1. Message count
			tim.msg_cnt = count_++;
			
			// 2. Data frame
			tim::msg::TravelerDataFrame traveler_data_frame;

			traveler_data_frame.set__not_used(0);
			traveler_data_frame.set__frame_type(1);
			traveler_data_frame.set__msg_id(0);
			traveler_data_frame.set__start_time( int(time.seconds()/60)%60 );	// [minutes]
			traveler_data_frame.set__duration_time(1.);							// [minutes]
			traveler_data_frame.set__priority(0);

			traveler_data_frame.set__not_used1(0);
			tim::msg::GeographicalPath region; {
				region.set__anchor_lat(0);		// [1/10th microdegree]
				region.set__anchor_long(0);		// [1/10th microdegree]
			}
			traveler_data_frame.regions.push_back(region);

			traveler_data_frame.set__not_used2(0);
			traveler_data_frame.set__not_used3(0);
			traveler_data_frame.set__content(0);

			tim.data_frames.push_back(traveler_data_frame);

			// 3. Regional
			tim::msg::RegionalExtension regional_extension; {

				// 3.1. Edge
				regional_extension.set__time_stamp( int(time.nanoseconds()/60)%60 );	// [milliseconds]
				regional_extension.set__processing_time(0);								// [milliseconds]
				tim::msg::Edge edge; {
					edge.set__id(id_);
					edge.set__coordinate_system(5186);						// EPSG
					edge.set__x(pos_x_);									// [m]
					edge.set__y(pos_y_);									// [m]
				}
				regional_extension.set__edge(edge);
				
				// 3.2. Objects
				regional_extension.set__num_objects(observation.size());
				for (Object & observed_object: observation_) {
					float x, y, Y;
					std::vector< std::tuple<float, float> > footprint;
					std::tie(x, y, Y, footprint) = observed_object.get_states();
					float dx = x - edge.x;
					float dy = y - edge.y;
					float dist = sqrt(pow(dx, 2) + pow(dy, 2));
					
					tim::msg::Object object; {

						// 3.2.1. State					
						object.set__id( observed_object.get_id() );
						object.pose.set__x( x*100. );						// [m --> cm]
						object.pose.set__y( y*100. );						// [m --> cm]
						object.pose.set__angle( Y*180./M_PI/0.0125 );		// [rad --> 0.0125deg]
						object.velocity.set__x( 0./0.02 );					// [m --> 0.02m]
						object.velocity.set__y( 0./0.02 );					// [m --> 0.02m]

						object.footprint.set__num_nodes(4);
						for (std::tuple<float, float> & node: footprint) {
							tim::msg::Node n;
							n.set__x( (std::get<0>(node) - edge.x)*100. );	// [m --> cm]
							n.set__y( (std::get<1>(node) - edge.y)*100. );	// [m --> cm]							
							object.footprint.nodes.push_back(n);
						}

						// 3.2.2. Classification					
						object.set__object(	// OBJECT_CAR for detected, OBJECT_UNKNOWN for undetected
							dist < detection_range_ ? tim::msg::Object::OBJECT_CAR : tim::msg::Object::OBJECT_UNKNOWN);					
						object.set__object_classification_score(90);
						object.set__location(tim::msg::Object::LOCATION_CARLANE);
						object.set__location_classification_score(90);
						object.set__num_actions(1);
						tim::msg::Action action; {
							action.set__action(tim::msg::Action::MOVING);
							action.set__action_classification_score(90);
						}
						object.actions.push_back(action);

						// 3.2.3. Trajectory forecasting
						object.trajectory_forecasting.set__prediction_horizon(0);
						object.trajectory_forecasting.set__sampling_period(0);
						object.trajectory_forecasting.set__num_predictions(1);
						tim::msg::Prediction prediction; {
							prediction.set__num_nodes(1);
							tim::msg::Node node; {
								node.set__x(object.pose.x);		// [cm]
								node.set__y(object.pose.y);		// [cm]
							}
							prediction.nodes.push_back(node);
						}
						object.trajectory_forecasting.predictions.push_back(prediction);
						object.set__trajectory_forecasting_score(90);
					}
					regional_extension.objects.push_back(object);
				}
			}
			tim.regionals.push_back(regional_extension);
		}
		return tim;
	}

	visualization_msgs::msg::MarkerArray generate_markers(tim::msg::TravelerInformationMessage & tim)
	{
		auto regional = tim.regionals[0];
		auto tim_visualization = visualization_msgs::msg::MarkerArray();
		float edge_height = 5.;

		// 3.1. Edge		
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
		tim_visualization.markers.push_back(edge_marker);

		// 3.2. Objects
		for (const auto & object: regional.objects) {
			visualization_msgs::msg::Marker object_marker;

			object_marker.header.set__frame_id( std::to_string(regional.edge.coordinate_system) );
			object_marker.set__id(object.id);
			for (const auto & node: object.footprint.nodes) {
				geometry_msgs::msg::Point point;
				point.set__x( node.x/100. + regional.edge.x );						// [cm --> m] & offset
				point.set__y( node.y/100. + regional.edge.y );						// [cm --> m] & offset
				object_marker.points.push_back(point);
			} {
				geometry_msgs::msg::Point point;
				point.set__x( object.footprint.nodes[0].x/100. + regional.edge.x );	// [cm --> m] & offset
				point.set__y( object.footprint.nodes[0].y/100. + regional.edge.y );	// [cm --> m] & offset
				object_marker.points.push_back(point);
			}
			
			object_marker.set__ns("object");
			object_marker.set__type(visualization_msgs::msg::Marker::LINE_STRIP);
			object_marker.set__action(visualization_msgs::msg::Marker::MODIFY);
			object_marker.scale.set__x(0.1);
			if (object.object == tim::msg::Object::OBJECT_UNKNOWN) {	// BLUE for undetected object
				object_marker.color.set__a(0.5);
				object_marker.color.set__r(0.);
				object_marker.color.set__g(0.);
				object_marker.color.set__b(1.);
			}
			else {														// RED for detected object
				object_marker.color.set__a(0.5);
				object_marker.color.set__r(1.);
				object_marker.color.set__g(0.);
				object_marker.color.set__b(0.);
			}			

			tim_visualization.markers.push_back(object_marker);
		}
        return tim_visualization;
	}

	rclcpp::Publisher<tim::msg::TravelerInformationMessage>::SharedPtr rsu;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher;
	rclcpp::TimerBase::SharedPtr timer;

	int id_;
	float pos_x_{ 10. };			// [m]
	float pos_y_{ -10. };			// [m]
	float detection_range_{ 30. };	// [m]
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
