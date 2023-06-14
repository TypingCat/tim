# States
uint32 id
Pose pose
Velocity velocity

# Footprint
Footprint footprint

# Classifications
## Object
uint8 OBJECT_UNKNOWN = 0
uint8 OBJECT_CAR = 1
uint8 OBJECT_TRUCK = 2
uint8 OBJECT_BUS = 3
uint8 OBJECT_CONSTRUCTION_VEHICLE = 4
uint8 OBJECT_MOTORCYCLE = 5
uint8 OBJECT_BICYCLE = 6
uint8 OBJECT_BICYCLE_RACK = 7
uint8 OBJECT_TRAILER = 8
uint8 OBJECT_PEDESTRIAN = 9
uint8 OBJECT_PUSHABLE_PULLABLE_OBJECT = 10
uint8 OBJECT_KICK_BOARD = 11
uint8 OBJECT_TRAFFIC_CONE = 12
uint8 OBJECT_TRAFFIC_BARRIER = 13
uint8 OBJECT_TRAFFIC_BARREL = 14
uint8 OBJECT_TRAFFIC_FENCE = 15
uint8 OBJECT_CONSTRUCTION_ZONE = 16

uint8 object
uint8 object_classification_score

## Location
uint8 LOCATION_UNKNOWN = 0
uint8 LOCATION_CARLANE = 1
uint8 LOCATION_PEDLANE = 2
uint8 LOCATION_SIDELANE = 3
uint8 LOCATION_PARKING = 4

uint8 location
uint8 location_classification_score

## Action
uint8 num_actions
Action[] actions

# Trajectory forecasting
TrajectoryForecasting trajectory_forecasting
uint8 trajectory_forecasting_score