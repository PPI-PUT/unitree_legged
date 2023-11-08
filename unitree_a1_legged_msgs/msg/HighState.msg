std_msgs/Header header
UnitreeHeader a1       
uint8 mode
sensor_msgs/Imu imu
float32 forward_speed
float32 side_speed
float32 rotate_speed
float32 body_height
float32 up_down_speed
float32 forward_position       # (will be float type next version)   # Old version Aliengo is different
float32 side_position          # (will be float type next version)   # Old version Aliengo is different
geometry_msgs/Point[4] foot_position_to_body
geometry_msgs/Point[4] foot_speed_to_body
int16[4] foot_force                  # Old version Aliengo is different
int16[4] foot_force_est               # Old version Aliengo does not have
uint32 tick               
uint8[40] wireless_remote
uint32 reserve                      # Old version Aliengo does not have
uint32 crc

# Under are not defined in SDK yet. # Old version Aliengo does not have
geometry_msgs/Point[4] ee_force            # It's a 1-DOF force in real robot, but 3-DOF is better for visualization.
float32[12] joint_pose              # for visualization