# This represents a traffic cone with a certain color, at a certain 3d position, not moving and facing upward
int32[] ids                  	# possible ids of the detected object (empty if no id could be identified/assigned)
int32 color		     	          # color of the cone
float64[] ids_confidence     	# confidence of the possible ids in [0, 1]
geometry_msgs/Point position  # position of the upward-facing cone

# cone type constants
int32 CONE_COLOR_UNKNOWN = 0
int32 CONE_COLOR_BLUE = 1
int32 CONE_COLOR_YELLOW = 2
int32 CONE_COLOR_RED = 3
