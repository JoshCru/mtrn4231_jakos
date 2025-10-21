# Weight estimation for an object
std_msgs/Header header
uint32 object_id
float32 estimated_weight  # in grams
float32 confidence       # 0.0 to 1.0
geometry_msgs/Pose pose
float32 volume           # in cubic cm
