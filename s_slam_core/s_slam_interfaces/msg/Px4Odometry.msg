# Source state for px4_msgs/VehicleOdometry. Pose and linear velocity are
# expressed in header.frame_id; body_frame identifies the pose reference point.
std_msgs/Header header
string body_frame

geometry_msgs/Pose pose
float64[36] pose_covariance

geometry_msgs/Vector3 linear_velocity
float64[9] linear_velocity_covariance

# The reset counter applies to this exact state sample.
uint8 reset_counter
