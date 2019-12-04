# Map unique ID
int16 id
bool is_submap

# Interval in which the map was active
time start_time
time end_time

# Current estimate of the pose of the map's base frame
voxgraph_msgs/MapPoseEstimate pose_estimate
