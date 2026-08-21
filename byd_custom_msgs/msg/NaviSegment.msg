uint8 segment_type
uint8 SEGMENT_TYPR_LINE = 1
uint8 SEGMENT_TYPR_BEZIER = 2
string segment_name
string segment_id
geometry_msgs/Point node1
geometry_msgs/Point node2
geometry_msgs/Point control_pos1
geometry_msgs/Point control_pos2
float32 max_load_speed
float32 max_speed
uint8 motion_direction
uint8 MOTION_DIRECTION_FORWARD = 1
uint8 MOTION_DIRECTION_BACKWARD = 2
uint32 dwell_time
