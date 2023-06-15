#include <iostream>
#include <vector>

// Components of TIM
namespace Tim {
    class GeographicalPath
    {
    public:
        GeographicalPath() { }

        int32_t lat;                    // [1/10th microdegree]
        int32_t lon;                    // [1/10th microdegree]
    };

    // Create TravelerDataFrame to comply with SAE J2735 standard
    // See https://www.sae.org/standards/content/j2735_202211/
    class TravelerDataFrame
    {
    public:
        TravelerDataFrame()
        {
            GeographicalPath anchor;
            regions.push_back(anchor);
        }

        // Part I, Frame header
        uint8_t not_used{ 0 };
        uint8_t frame_type{ 1 };        // advisory(1)
        uint16_t msg_id;                // variable type: octet string(2byte)
        uint32_t start_time;            // [minutes]
        uint16_t duration_time;         // [minutes]
        uint8_t priority;

        // Part II, Applicable Regions of Use
        uint8_t not_used1{ 0 };
        std::vector<GeographicalPath> regions;

        // Part III, Content
        uint8_t not_used2{ 0 };
        uint8_t not_used3{ 0 };
        uint16_t content;               // ITIS
    };

///////////////////////////////////////////////////////////////////////////////////////

    class Edge
    {
    public:
        Edge() { }

        uint32_t id;
        uint16_t coordinate_system;     // EPSG
        _Float64 x;                     // [m]
        _Float64 y;                     // [m]
    };

    class Pose
    {
    public:
        Pose() { }

        int16_t x;                      // [cm]
        int16_t y;                      // [cm]
        uint16_t angle;                 // [0.0125deg]
    };

    class Velocity
    {
    public:
        Velocity() { }

        int16_t x;                      // [0.02m/s]
        int16_t y;                      // [0.02m/s]
    };

    class Node
    {
    public:
        Node() { }

        int16_t x;                      // [cm]
        int16_t y;                      // [cm]
    };

    class Footprint
    {
    public:
        Footprint() { }

        uint8_t num_nodes;
        std::vector<Node> nodes;
    };

    class Action
    {
    public:
        Action() { }

        uint8_t action;                 // enumerated in TIM::ACTION
        uint8_t action_classification_score;
    };

    class Prediction
    {
    public:
        Prediction() { }

        uint8_t num_nodes;
        std::vector<Node> nodes;
    };

    class TrajectoryForecasting
    {
    public:
        TrajectoryForecasting() { }

        uint16_t prediction_horizon;    // [milliseconds]
        uint16_t sampling_period;       // [milliseconds]
        uint8_t num_predictions;
        std::vector<Prediction> predictions;
    };

    class Object
    {
    public:
        Object() { }

        // States
        uint32_t id;
        Pose pose;
        Velocity velocity;

        // Footprint
        Footprint footprint;

        // Classifications
        uint8_t object;                 // enumerated in TIM::OBJECT
        uint8_t object_classification_score;

        uint8_t location;               // enumerated in TIM::LOCATION
        uint8_t location_classification_score;

        uint8_t num_actions;
        std::vector<Action> actions;

        // Trajectory forecasting
        TrajectoryForecasting trajectory_forecasting;
        uint8_t trajectory_forecasting_score;
    };

    // Define RegionalExtension to cooperative perception between infra and vehicles
    // Mainly provides the status and classification of objects
    class RegionalExtension
    {
    public:
        RegionalExtension() { }
        
        // 3.1. Edge
        uint16_t time_stamp;            // [milliseconds]
        uint16_t processing_time;       // [milliseconds]
        Edge edge;

        // 3.2. Objects
        uint8_t num_objects;
        std::vector<Object> objects;
    };

    namespace OBJECT
    {
        uint8_t UNKNOWN = 0;
        uint8_t CAR = 1;
        uint8_t TRUCK = 2;
        uint8_t BUS = 3;
        uint8_t CONSTRUCTION_VEHICLE = 4;
        uint8_t MOTORCYCLE = 5;
        uint8_t BICYCLE = 6;
        uint8_t BICYCLE_RACK = 7;
        uint8_t TRAILER = 8;
        uint8_t PEDESTRIAN = 9;
        uint8_t PUSHABLE_PULLABLE_OBJECT = 10;
        uint8_t KICK_BOARD = 11;
        uint8_t TRAFFIC_CONE = 12;
        uint8_t TRAFFIC_BARRIER = 13;
        uint8_t TRAFFIC_BARREL = 14;
        uint8_t TRAFFIC_FENCE = 15;
        uint8_t CONSTRUCTION_ZONE = 16;
    }

    namespace LOCATION
    {
        uint8_t UNKNOWN = 0;
        uint8_t CARLANE = 1;
        uint8_t PEDLANE = 2;
        uint8_t SIDELANE = 3;
        uint8_t PARKING = 4;
    }

    namespace ACTION
    {
        uint8_t UNKNOWN = 0;
        uint8_t MOVING = 1;
        uint8_t STOP = 2;
        uint8_t STABLE = 3;
        uint8_t EMERGENCY = 4;
        uint8_t ACCIDENT = 5;
    }
}

///////////////////////////////////////////////////////////////////////////////////////

// Traveler Information Message (TIM)
// Communication Protocal Design Specification for Connected Autonomous Driving Service Edge AI v0.5
class TIM
{
public:
    TIM(uint8_t id=0)
    {
        msgCnt = id;

        Tim::TravelerDataFrame traveler_data_frame;
        data_frames.push_back(traveler_data_frame);

        Tim::RegionalExtension regional_extension;
        regionals.push_back(regional_extension);
    }

    uint8_t msgCnt;
    std::vector<Tim::TravelerDataFrame> data_frames;
    std::vector<Tim::RegionalExtension> regionals;
};