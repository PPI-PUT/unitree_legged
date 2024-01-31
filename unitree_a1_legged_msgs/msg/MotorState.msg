uint8 mode           # motor current mode 
float32 q            # motor current position（rad）
float32 dq           # motor current speed（rad/s）
# float32 ddq          # motor current acceleration（rad/s^2）always 0
float32 tau_est      # current estimated output torque（N*m）
# float32 q_raw        # motor current position（rad）always 0
# float32 dq_raw       # motor current speed（rad/s） always 0
# float32 ddq_raw      # motor current acceleration（rad/s^2）always 0
int8 temperature     # motor temperature（slow conduction of temperature leads to lag）
# uint32[2] reserve  # reserve always 0