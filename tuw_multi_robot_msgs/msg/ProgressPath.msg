#################################################
# Reports the progress a robot made along a local 
# path.
#################################################

Header header
nav_msgs/Path progress      # portion of the path which is already processed by the robot
nav_msgs/Path ahead         # portion of the path which is ahead of the robot
float32 progress_metric     # indicator showing the progress in %
int32 step                  # indicator showing the current step of the plan
int32 total_steps           # total number of steps
