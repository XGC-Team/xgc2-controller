uint8 OBJECTIVE_MIN_SNAP = 1

std_msgs/Header header
uint32 request_id
uint32 trajectory_id
uint32 flags
geometry_msgs/Pose[] waypoints
float64[] segment_times
geometry_msgs/Vector3 start_velocity
geometry_msgs/Vector3 start_acceleration
geometry_msgs/Vector3 end_velocity
geometry_msgs/Vector3 end_acceleration
float64 max_velocity
float64 max_acceleration
float64 max_jerk
float64 max_snap
uint8 objective
