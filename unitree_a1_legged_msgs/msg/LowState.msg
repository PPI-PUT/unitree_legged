std_msgs/Header header
# UnitreeHeader a1 # not used             
sensor_msgs/Imu imu
QuadrupedState motor_state
FootForceState foot_force             # force sensors
# FootForceState foot_force_est           # force senors for A1 is always 0
uint32 tick                     # reference real-time from motion controller (unit: us)
# uint8[40] wireless_remote        # wireless commands
# uint32 reserve                  
# uint32 crc    
