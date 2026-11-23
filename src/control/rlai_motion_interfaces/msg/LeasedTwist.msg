# Token returned by acquire; never put it in status, logs or browser state.
string lease_id
uint64 sequence
# Host SYSTEM/UTC time, not ROS simulation time. Delayed commands expire at receipt.
builtin_interfaces/Time issued_at
uint32 valid_for_ms
geometry_msgs/Twist twist
