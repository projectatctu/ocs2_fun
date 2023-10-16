# Joint command message
#
# This message contains the name of controlled joint, feed-forward torque,
# desired joint position and velocity together with the kp and kd
# values for PD controller.
#
# The final control signal may be computed as
# torque = torque_ff + kp * pos_error + kd * vel_error

string joint_name
float64 position_desired
float64 velocity_desired
float64 kp
float64 kd
float64 torque_ff