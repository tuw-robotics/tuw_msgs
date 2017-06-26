Header header

# requested speed on the center of the rear axis [m/s]
float64 vel

# requested steering angle of the steering wheel [degrees]
float64 steering_angle

# requested brake pressure for the system in bar measured at the front brake circuit
# if > 0, vel is being overriden and 0-torque is applied on the motors
float64 brake_pressure
