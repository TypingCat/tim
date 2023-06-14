# Part I, Frame header
uint8 not_used          # set to 0
uint8 frame_type        # set to advisory(1)
uint16 msg_id
uint32 start_time       # [minutes]
uint16 duration_time    # [minutes]
uint8 priority

# Part II, Applicable Regions of Use
uint8 not_used1         # set to 0
GeographicalPath[] regions

# Part III, Content
uint8 not_used2         # set to 0
uint8 not_used3         # set to 0
uint16 content          # ITIS