# A single frame from a downward-facing thermal sensor on a drone.
# Pose is embedded so the consumer (mapper) does not need to time-sync against /cfN/odom.

std_msgs/Header header
geometry_msgs/Pose pose
float32 footprint_size
uint16 width
uint16 height
float32[] data
