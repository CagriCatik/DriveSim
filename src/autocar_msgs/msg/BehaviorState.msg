uint8 LANE_FOLLOW=0
uint8 STOP=1
uint8 AVOID_OBSTACLE=2
uint8 GOAL_REACHED=3
uint8 EMERGENCY=4

std_msgs/Header header
uint8 state
string state_label
float64 target_velocity
bool emergency
