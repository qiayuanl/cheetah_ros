uint8 STAND=0
uint8 SWING=1

Header header
LegPrefix[] leg_prefix
uint8[] touch_state

# STAND
geometry_msgs/Vector3[] ground_reaction_force

# SWING
geometry_msgs/Point[] pos_final
float64[] height
float64[] swing_time
