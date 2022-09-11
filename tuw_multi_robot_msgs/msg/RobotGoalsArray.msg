#################################################################
## Used to send goal requests for multiple robots to the multi robot planner
#################################################################

std_msgs/Header header  # the time of the generation
RobotGoals[] robots     # contains a goals/checkpoints for each robot
