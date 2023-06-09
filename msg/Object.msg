# States
uint32 id
int16 pose_x
int16 pose_y
uint16 pose_angle
int16 velocity_x
int16 velocity_y

# Footprint
uint8 footprint_num_nodes
Node[] footprint_nodes

# Classifications
uint8 object
uint8 object_classification_score
uint8 location
uint8 location_classification_score
uint8 num_actions
Action[] action

# Trajectory forecasting
uint16 trajectory_forecasting_prediction_horizon
uint16 trajectory_forecasting_sampling_period
uint8 trajectory_forecasting_num_predictions
Prediction[] trajectory_forecasting_predictions
uint8 trajectory_forecasting_score