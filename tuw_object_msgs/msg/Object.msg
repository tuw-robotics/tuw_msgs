# This expresses a object in a 3D space
int32[] ids                  # possible ids of the detected object (empty if no id could be identified/assigned)
float64[] ids_confidence     # confidence of the possible ids in [0, 1]
geometry_msgs/Pose pose      # pose of the detected object
geometry_msgs/Twist twist    # twist of the detected object