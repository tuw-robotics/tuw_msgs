Header header
# control type
uint8 TORQUE_CONTROL=0
uint8 SPEED_CONTROL=1
uint8 control_type
# requested torque on the left rear wheel in Nm
float64 left_torque
# requested torque on the right rear wheel in Nm
float64 right_torque
# requested speed on the left rear wheel in m/s
float64 left_speed
# requested speed on the right rear wheel in m/s
float64 right_speed
# requested steering angle in degrees
float64 steering_angle
# requested brake pressure for the system in bar measured at the front brake circuit
float64 brake_pressure
