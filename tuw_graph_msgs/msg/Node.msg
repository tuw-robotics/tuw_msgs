#################################################################
## A single node in a graph
## Each node of the same graph must have a unique id.
#################################################################
int64 id                     # Node id
bool valid                   # True if it can be used for planning
geometry_msgs/Pose pose      # Position
int32[] flags                # Flags or attributes
 
