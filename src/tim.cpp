#include "tim.hpp"

// Create empty TIM
TIM::TIM()
{
    Tim::TravelerDataFrame traveler_data_frame;
    data_frames.push_back(traveler_data_frame);

    Tim::RegionalExtension regional_extension;
    regionals.push_back(regional_extension);
}

// Convert ROS message to TIM
TIM::TIM(const tim::msg::TravelerInformationMessage & rosmsg)
{
    // 1. Message count
    msgCnt = rosmsg.msg_cnt;

    // 2. Data frame
    Tim::TravelerDataFrame tim_traveler_data_frame; {
        tim_traveler_data_frame.not_used = rosmsg.data_frames[0].not_used;
        tim_traveler_data_frame.frame_type = rosmsg.data_frames[0].frame_type;
        tim_traveler_data_frame.msg_id = rosmsg.data_frames[0].msg_id;
        tim_traveler_data_frame.start_time = rosmsg.data_frames[0].start_time;
        tim_traveler_data_frame.duration_time = rosmsg.data_frames[0].duration_time;
        tim_traveler_data_frame.priority = rosmsg.data_frames[0].priority;

        tim_traveler_data_frame.not_used1 = rosmsg.data_frames[0].not_used1;
        Tim::GeographicalPath tim_region; {
            tim_region.lat = rosmsg.data_frames[0].regions[0].anchor_lat;
            tim_region.lon = rosmsg.data_frames[0].regions[0].anchor_lon;
        }
        tim_traveler_data_frame.regions.push_back(tim_region);

        tim_traveler_data_frame.not_used2 = rosmsg.data_frames[0].not_used2;
        tim_traveler_data_frame.not_used3 = rosmsg.data_frames[0].not_used3;
        tim_traveler_data_frame.content = rosmsg.data_frames[0].content;
    }
    data_frames.push_back(tim_traveler_data_frame);

    // 3. Regional
    Tim::RegionalExtension tim_regional_extension; {

        // 3.1. Edge
        tim_regional_extension.time_stamp = rosmsg.regionals[0].time_stamp;
        tim_regional_extension.processing_time = rosmsg.regionals[0].processing_time;
        tim_regional_extension.edge.id = rosmsg.regionals[0].edge.id;
        tim_regional_extension.edge.coordinate_system = rosmsg.regionals[0].edge.coordinate_system;
        tim_regional_extension.edge.x = rosmsg.regionals[0].edge.x;
        tim_regional_extension.edge.y = rosmsg.regionals[0].edge.y;
    
        // 3.2. Objects
        tim_regional_extension.num_objects = rosmsg.regionals[0].num_objects;
        for (const tim::msg::Object & object: rosmsg.regionals[0].objects) {
            Tim::Object tim_object; {

                // 3.2.1. State
                tim_object.id = object.id;
                tim_object.pose.x = object.pose.x;
                tim_object.pose.y = object.pose.y;
                tim_object.pose.angle = object.pose.angle;
                tim_object.velocity.x = object.velocity.x;
                tim_object.velocity.y = object.velocity.y;

                tim_object.footprint.num_nodes = object.footprint.num_nodes;
                for (const tim::msg::Node & node: object.footprint.nodes) {
                    Tim::Node tim_node; {
                        tim_node.x = node.x;
                        tim_node.y = node.y;
                    }
                    tim_object.footprint.nodes.push_back(tim_node);
                }

                // 3.2.2. Classification
                tim_object.object = object.object;
                tim_object.object_classification_score = object.object_classification_score;
                tim_object.location = object.location;
                tim_object.location_classification_score = object.location_classification_score;
                tim_object.num_actions = object.num_actions;
                for (const tim::msg::Action & action: object.actions) {
                    Tim::Action tim_action; {
                        tim_action.action = action.action;
                        tim_action.action_classification_score = action.action_classification_score;
                    }
                    tim_object.actions.push_back(tim_action);
                }

                // 3.2.3. Trajectory forecasting
                tim_object.trajectory_forecasting.prediction_horizon = object.trajectory_forecasting.prediction_horizon;
                tim_object.trajectory_forecasting.sampling_period = object.trajectory_forecasting.sampling_period;
                tim_object.trajectory_forecasting.num_predictions = object.trajectory_forecasting.num_predictions;
                for (const tim::msg::Prediction & prediction: object.trajectory_forecasting.predictions) {
                    Tim::Prediction tim_prediction; {
                        tim_prediction.num_nodes = prediction.num_nodes;
                        for (const tim::msg::Node & node: prediction.nodes) {
                            Tim::Node tim_node; {
                                tim_node.x = node.x;
                                tim_node.y = node.y;
                            }
                            tim_prediction.nodes.push_back(tim_node);
                        }
                    }
                    tim_object.trajectory_forecasting.predictions.push_back(tim_prediction);
                }
                tim_object.trajectory_forecasting_score = object.trajectory_forecasting_score;
            }
            tim_regional_extension.objects.push_back(tim_object);
        }
    }
    regionals.push_back(tim_regional_extension);
}

// Convert string to TIM
TIM::TIM(const std::string & s)
{
    std::stringstream ss(s);

    // 1. Message count
    read_buffer(ss, msgCnt);

    // 2. Data frame
    Tim::TravelerDataFrame tim_traveler_data_frame; {
        read_buffer(ss, tim_traveler_data_frame.not_used);
        read_buffer(ss, tim_traveler_data_frame.frame_type);
        read_buffer(ss, tim_traveler_data_frame.msg_id);
        read_buffer(ss, tim_traveler_data_frame.start_time);
        read_buffer(ss, tim_traveler_data_frame.duration_time);
        read_buffer(ss, tim_traveler_data_frame.priority);
        
        read_buffer(ss, tim_traveler_data_frame.not_used1);
        Tim::GeographicalPath tim_region; {
            read_buffer(ss, tim_region.lat);
            read_buffer(ss, tim_region.lon);
        }
        tim_traveler_data_frame.regions.push_back(tim_region);

        read_buffer(ss, tim_traveler_data_frame.not_used2);
        read_buffer(ss, tim_traveler_data_frame.not_used3);
        read_buffer(ss, tim_traveler_data_frame.content);
    }
    data_frames.push_back(tim_traveler_data_frame);

    // 3. Regional
    Tim::RegionalExtension tim_regional_extension; {

        // 3.1. Edge
        read_buffer(ss, tim_regional_extension.time_stamp);
        read_buffer(ss, tim_regional_extension.processing_time);
        read_buffer(ss, tim_regional_extension.edge.id);
        read_buffer(ss, tim_regional_extension.edge.coordinate_system);
        read_buffer(ss, tim_regional_extension.edge.x);
        read_buffer(ss, tim_regional_extension.edge.y);
    
        // 3.2. Objects
        read_buffer(ss, tim_regional_extension.num_objects);
        for (int object=0; object<tim_regional_extension.num_objects; ++object) {
            Tim::Object tim_object; {

                // 3.2.1. State
                read_buffer(ss, tim_object.id);
                read_buffer(ss, tim_object.pose.x);
                read_buffer(ss, tim_object.pose.y);
                read_buffer(ss, tim_object.pose.angle);
                read_buffer(ss, tim_object.velocity.x);
                read_buffer(ss, tim_object.velocity.y);

                read_buffer(ss, tim_object.footprint.num_nodes);
                for (int footprint_node=0; footprint_node<tim_object.footprint.num_nodes; ++footprint_node) {
                    Tim::Node tim_node; {
                        read_buffer(ss, tim_node.x);
                        read_buffer(ss, tim_node.y);
                    }
                    tim_object.footprint.nodes.push_back(tim_node);
                }

                // 3.2.2. Classification
                read_buffer(ss, tim_object.object);
                read_buffer(ss, tim_object.object_classification_score);
                read_buffer(ss, tim_object.location);
                read_buffer(ss, tim_object.location_classification_score);
                read_buffer(ss, tim_object.num_actions);
                for (int action=0; action<tim_object.num_actions; ++action) {
                    Tim::Action tim_action; {
                        read_buffer(ss, tim_action.action);
                        read_buffer(ss, tim_action.action_classification_score);
                    }
                    tim_object.actions.push_back(tim_action);
                }

                // 3.2.3. Trajectory forecasting
                read_buffer(ss, tim_object.trajectory_forecasting.prediction_horizon);
                read_buffer(ss, tim_object.trajectory_forecasting.sampling_period);
                read_buffer(ss, tim_object.trajectory_forecasting.num_predictions);
                for (int prediction=0; prediction<tim_object.trajectory_forecasting.num_predictions; ++prediction) {
                    Tim::Prediction tim_prediction; {
                        read_buffer(ss, tim_prediction.num_nodes);
                        for (int prediction_node=0; prediction_node<tim_prediction.num_nodes; ++prediction_node) {
                            Tim::Node tim_node; {
                                read_buffer(ss, tim_node.x);
                                read_buffer(ss, tim_node.y);
                            }
                            tim_prediction.nodes.push_back(tim_node);
                        }
                    }
                    tim_object.trajectory_forecasting.predictions.push_back(tim_prediction);
                }
                read_buffer(ss, tim_object.trajectory_forecasting_score);
            }
            tim_regional_extension.objects.push_back(tim_object);
        }
    }
    regionals.push_back(tim_regional_extension);
}

// Convert TIM to ROS message
tim::msg::TravelerInformationMessage TIM::to_rosmsg()
{
    tim::msg::TravelerInformationMessage rosmsg; {

        // 1. Message count
        rosmsg.msg_cnt = msgCnt;

        // 2. Data frame
        tim::msg::TravelerDataFrame rosmsg_traveler_data_frame; {
            rosmsg_traveler_data_frame.set__not_used(data_frames[0].not_used);
            rosmsg_traveler_data_frame.set__frame_type(data_frames[0].frame_type);
            rosmsg_traveler_data_frame.set__msg_id(data_frames[0].msg_id);
            rosmsg_traveler_data_frame.set__start_time(data_frames[0].start_time);          // [minutes]
            rosmsg_traveler_data_frame.set__duration_time(data_frames[0].duration_time);    // [minutes]
            rosmsg_traveler_data_frame.set__priority(data_frames[0].priority);

            rosmsg_traveler_data_frame.set__not_used1(data_frames[0].not_used1);
            tim::msg::GeographicalPath rosmsg_region; {
                rosmsg_region.set__anchor_lat(data_frames[0].regions[0].lat);	// [1/10th microdegree]
                rosmsg_region.set__anchor_lon(data_frames[0].regions[0].lon);	// [1/10th microdegree]
            }
            rosmsg_traveler_data_frame.regions.push_back(rosmsg_region);

            rosmsg_traveler_data_frame.set__not_used2(data_frames[0].not_used2);
            rosmsg_traveler_data_frame.set__not_used3(data_frames[0].not_used3);
            rosmsg_traveler_data_frame.set__content(data_frames[0].content);
        }
        rosmsg.data_frames.push_back(rosmsg_traveler_data_frame);

        // 3. Regional
        tim::msg::RegionalExtension rosmsg_regional_extension; {
            
            // 3.1. Edge
            rosmsg_regional_extension.set__time_stamp(regionals[0].time_stamp);	            // [milliseconds]
            rosmsg_regional_extension.set__processing_time(regionals[0].processing_time);	// [milliseconds]
            tim::msg::Edge rosmsg_edge; {
                rosmsg_edge.set__id(regionals[0].edge.id);
                rosmsg_edge.set__coordinate_system(regionals[0].edge.coordinate_system);	// EPSG
                rosmsg_edge.set__x(regionals[0].edge.x);					                // [m]
                rosmsg_edge.set__y(regionals[0].edge.y);					                // [m]
            }
            rosmsg_regional_extension.set__edge(rosmsg_edge);

            // 3.2. Objects
            rosmsg_regional_extension.set__num_objects(regionals[0].num_objects);
            for (Tim::Object & object: regionals[0].objects) {
                tim::msg::Object rosmsg_object; {

                    // 3.2.1. State
                    rosmsg_object.set__id(object.id);
                    rosmsg_object.pose.set__x(object.pose.x);				// [cm]
                    rosmsg_object.pose.set__y(object.pose.y);				// [cm]
                    rosmsg_object.pose.set__angle(object.pose.angle);		// [0.0125deg]
                    rosmsg_object.velocity.set__x(object.velocity.x);		// [0.02m]
                    rosmsg_object.velocity.set__y(object.velocity.y);		// [0.02m]

                    rosmsg_object.footprint.set__num_nodes(object.footprint.num_nodes);
                    for (Tim::Node & node: object.footprint.nodes) {
                        tim::msg::Node rosmsg_node; {
                            rosmsg_node.set__x(node.x);	    // [cm]
                            rosmsg_node.set__y(node.y);	    // [cm]
                        }
                        rosmsg_object.footprint.nodes.push_back(rosmsg_node);
                    }

                    // 3.2.2. Classification
                    rosmsg_object.set__object(object.object);					
                    rosmsg_object.set__object_classification_score(object.object_classification_score);
                    rosmsg_object.set__location(object.location);
                    rosmsg_object.set__location_classification_score(object.location_classification_score);
                    rosmsg_object.set__num_actions(object.num_actions);
                    for (Tim::Action & action: object.actions) {
                        tim::msg::Action rosmsg_action; {
                            rosmsg_action.set__action(action.action);
                            rosmsg_action.set__action_classification_score(action.action_classification_score);
                        }
                        rosmsg_object.actions.push_back(rosmsg_action);
                    }

                    // 3.2.3. Trajectory forecasting
                    rosmsg_object.trajectory_forecasting.set__prediction_horizon(object.trajectory_forecasting.prediction_horizon);
                    rosmsg_object.trajectory_forecasting.set__sampling_period(object.trajectory_forecasting.sampling_period);
                    rosmsg_object.trajectory_forecasting.set__num_predictions(object.trajectory_forecasting.num_predictions);
                    for (Tim::Prediction & prediction: object.trajectory_forecasting.predictions) {
                        tim::msg::Prediction rosmsg_prediction; {
                            rosmsg_prediction.set__num_nodes(prediction.num_nodes);
                            for (Tim::Node & node: prediction.nodes) {
                                tim::msg::Node rosmsg_node; {
                                    rosmsg_node.set__x(node.x);	// [cm]
                                    rosmsg_node.set__y(node.y);	// [cm]
                                }
                                rosmsg_prediction.nodes.push_back(rosmsg_node);
                            }
                        }
                        rosmsg_object.trajectory_forecasting.predictions.push_back(rosmsg_prediction);
                    }
                    rosmsg_object.set__trajectory_forecasting_score(object.trajectory_forecasting_score);
                }
                rosmsg_regional_extension.objects.push_back(rosmsg_object);
            }
        }
        rosmsg.regionals.push_back(rosmsg_regional_extension);
    }
    return rosmsg;
}

// Convert TIM to Rviz message
visualization_msgs::msg::MarkerArray TIM::to_rviz()
{
    float edge_height = 5.;
    visualization_msgs::msg::MarkerArray rviz; {
    
        // Edge
		visualization_msgs::msg::Marker edge_marker; {
			edge_marker.header.set__frame_id(std::to_string(regionals[0].edge.coordinate_system));
			edge_marker.set__id(regionals[0].edge.id);
			edge_marker.pose.position.set__x(regionals[0].edge.x);	// [m]
			edge_marker.pose.position.set__y(regionals[0].edge.y);	// [m]
			edge_marker.pose.position.set__z(edge_height/2.);	    // offset
			
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
            edge_marker.lifetime.set__sec(0);
            edge_marker.lifetime.set__nanosec(100000000);
		}
        rviz.markers.push_back(edge_marker);

        // Objects
        for (auto & object: regionals[0].objects) {
			visualization_msgs::msg::Marker object_marker; {
                object_marker.header.set__frame_id(std::to_string(regionals[0].edge.coordinate_system));
                object_marker.set__id(object.id);
                for (const auto & node: object.footprint.nodes) {
                    geometry_msgs::msg::Point object_point;
                    object_point.set__x( node.x/100. + regionals[0].edge.x );						// [cm --> m] & offset
                    object_point.set__y( node.y/100. + regionals[0].edge.y );						// [cm --> m] & offset
                    object_marker.points.push_back(object_point);
                } {
                    geometry_msgs::msg::Point object_point;
                    object_point.set__x( object.footprint.nodes[0].x/100. + regionals[0].edge.x );	// [cm --> m] & offset
                    object_point.set__y( object.footprint.nodes[0].y/100. + regionals[0].edge.y );	// [cm --> m] & offset
                    object_marker.points.push_back(object_point);
                }
                
                object_marker.set__ns("object");
                object_marker.set__type(visualization_msgs::msg::Marker::LINE_STRIP);
                object_marker.set__action(visualization_msgs::msg::Marker::MODIFY);
                object_marker.scale.set__x(0.1);
                if (object.object == tim::msg::Object::OBJECT_UNKNOWN) {
                    object_marker.color.set__a(0.5);
                    object_marker.color.set__r(0.);
                    object_marker.color.set__g(0.);
                    object_marker.color.set__b(1.);
                }
                else {
                    object_marker.color.set__a(0.5);
                    object_marker.color.set__r(1.);
                    object_marker.color.set__g(0.);
                    object_marker.color.set__b(0.);
                }
                object_marker.lifetime.set__sec(0);
                object_marker.lifetime.set__nanosec(100000000);
            }
            rviz.markers.push_back(object_marker);
		}
    }
    return rviz;
}

// Convert TIM to string
std::string TIM::to_string()
{
    std::stringstream ss; {

        // 1. Message count
        write_buffer(ss, msgCnt);
        
        // 2. Data frame
        write_buffer(ss, data_frames[0].not_used);
        write_buffer(ss, data_frames[0].frame_type);
        write_buffer(ss, data_frames[0].msg_id);
        write_buffer(ss, data_frames[0].start_time);
        write_buffer(ss, data_frames[0].duration_time);
        write_buffer(ss, data_frames[0].priority);

        write_buffer(ss, data_frames[0].not_used1);
        write_buffer(ss, data_frames[0].regions[0].lat);
        write_buffer(ss, data_frames[0].regions[0].lon);

        write_buffer(ss, data_frames[0].not_used2);
        write_buffer(ss, data_frames[0].not_used3);
        write_buffer(ss, data_frames[0].content);

        // 3. Regional            
        // 3.1. Edge
        write_buffer(ss, regionals[0].time_stamp);
        write_buffer(ss, regionals[0].processing_time);
        write_buffer(ss, regionals[0].edge.id);
        write_buffer(ss, regionals[0].edge.coordinate_system);
        write_buffer(ss, regionals[0].edge.x);
        write_buffer(ss, regionals[0].edge.y);

        // 3.2. Objects
        write_buffer(ss, regionals[0].num_objects);
        for (Tim::Object & object: regionals[0].objects) {

            // 3.2.1. State
            write_buffer(ss, object.id);
            write_buffer(ss, object.pose.x);
            write_buffer(ss, object.pose.y);
            write_buffer(ss, object.pose.angle);
            write_buffer(ss, object.velocity.x);
            write_buffer(ss, object.velocity.y);
            write_buffer(ss, object.footprint.num_nodes);
            for (Tim::Node & node: object.footprint.nodes) {
                write_buffer(ss, node.x);
                write_buffer(ss, node.y);
            }

            // 3.2.2. Classification
            write_buffer(ss, object.object);
            write_buffer(ss, object.object_classification_score);
            write_buffer(ss, object.location);
            write_buffer(ss, object.location_classification_score);
            write_buffer(ss, object.num_actions);
            for (Tim::Action & action: object.actions) {
                write_buffer(ss, action.action);
                write_buffer(ss, action.action_classification_score);
            }

            // 3.2.3. Trajectory forecasting
            write_buffer(ss, object.trajectory_forecasting.prediction_horizon);
            write_buffer(ss, object.trajectory_forecasting.sampling_period);
            write_buffer(ss, object.trajectory_forecasting.num_predictions);
            for (Tim::Prediction & prediction: object.trajectory_forecasting.predictions) {
                write_buffer(ss, prediction.num_nodes);
                for (Tim::Node & node: prediction.nodes) {
                    write_buffer(ss, node.x);
                    write_buffer(ss, node.y);
                }
            }
            write_buffer(ss, object.trajectory_forecasting_score);
        }
    }
    return ss.str();
}

template <typename T> void TIM::write_buffer(std::stringstream & ss, T data)
{
    if (typeid(T) == typeid(uint8_t)) {
        auto temp = uint16_t(data);
        ss << temp << " ";
    }
    else {
        ss << data << " ";
    }
}

template <typename T> void TIM::read_buffer(std::stringstream & ss, T & data)
{
    if (typeid(T) == typeid(uint8_t)) {
        uint16_t temp;
        ss >> temp;
        data = T(temp);
    }
    else {
        ss >> data;
    }
}