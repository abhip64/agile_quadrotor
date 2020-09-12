# reference for polynomial trajectory tracking

std_msgs/Header header

uint8 type_mask
uint8 IGNORE_ALL      = 0	# Ignore all Refernce
uint8 IGNORE_NONE     = 1	# Position Velocity Acceleration Angular_Velocity Yaw Reference
uint8 IGNORE_POSITION = 2	# Velocity Acceleration Angular_Velocity Yaw Reference
uint8 IGNORE_VELOCITY = 3   # Position Acceleration Angular_Velocity Yaw Reference
uint8 IGNORE_POS_VEL  = 4	# Acceleration Angular_Velocity Yaw Reference

geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 ang_vel
std_msgs/Float64 yaw