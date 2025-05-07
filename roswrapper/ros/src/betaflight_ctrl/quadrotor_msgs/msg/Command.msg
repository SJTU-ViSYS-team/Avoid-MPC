Header header
uint8 mode
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 angularVel
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 jerk
geometry_msgs/Quaternion quat

float64 yaw
float64 yaw_dot
float64 thrust

uint8 POSITION_MODE = 0
uint8 ACCELERATION_MODE = 1
uint8 ANGULAR_MODE = 2
uint8 QUAT_MODE = 3