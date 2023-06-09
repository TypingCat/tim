# Part I, Frame header
uint8 not_used
uint8 frame_type
uint16 msg_id
uint32 start_time
uint16 duration_time
uint8 priority

# Part II, Applicable Regions of Use
uint8 not_used1
int32 anchor_lat
int32 anchor_long

# Part III, Content
uint8 not_used2
uint8 not_used3
uint16 advisory