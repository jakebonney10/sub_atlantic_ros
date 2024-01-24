std_msgs/Header header
int32 valve_id              # ID of the valve
bool is_on                  # Whether the valve is on or off
bool is_open_loop_control   # True for open loop control, false for closed loop
float64 current_set_point   # Current set point of the valve
