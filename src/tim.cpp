#include "tim.hpp"

Tim::TravelerDataFrame::TravelerDataFrame()
{
    GeographicalPath anchor;
    regions.push_back(anchor);
}

TIM::TIM()
{
    Tim::TravelerDataFrame traveler_data_frame;
    data_frames.push_back(traveler_data_frame);

    Tim::RegionalExtension regional_extension;
    regionals.push_back(regional_extension);
}

tim::msg::TravelerInformationMessage TIM::to_rosmsg()
{
    auto rosmsg = tim::msg::TravelerInformationMessage(); {

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
                        tim::msg::Node rosmsg_node;
                        rosmsg_node.set__x(node.x);	    // [cm]
                        rosmsg_node.set__y(node.y);	    // [cm]							
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
                                    rosmsg_node.set__x(rosmsg_object.pose.x);	// [cm]
                                    rosmsg_node.set__y(rosmsg_object.pose.y);	// [cm]
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

visualization_msgs::msg::MarkerArray TIM::to_rviz()
{
    auto marker_array = visualization_msgs::msg::MarkerArray(); {

        // Edge

        // Objects

        
    }
    return marker_array;
}