uint8 mode # to set for all motors same mode
uint8 control_mode # postion or torque ( if torque then q, dq, kp, kd are ignored)
float32 kp # if joints dont have same kp, set 0
float32 kd # if joints dont have same kd, set 0