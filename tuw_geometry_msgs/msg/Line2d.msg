# Line in 2D
# The line function is represented by alpha and rho as Hesse normal form 
# x cos(alpha) + y sin(alpha) = rho
uint32 id                     # id
float64 alpha                 # Angle of the normal to the line
float64 rho                   # Shortest distance from the origin to the line
geometry_msgs/Point[] p       # optinal data: points supporting the line