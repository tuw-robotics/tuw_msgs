#################################################################
## A single node in a graph
## Each node of the same graph must have a unique id.
#################################################################
int64 id                     # Node id
bool valid                   # True if it can be used for planning
geometry_msgs/Pose pose      # Position
uint32[] flags               # Flags or attributes
 

uint32 FLAG_NA = 0
uint32 FLAG_UNTOUCHED = 1
uint32 FLAG_OPEN = 2
uint32 FLAG_CLOSED = 3
uint32 FLAG_VISITED = 4
uint32 FLAG_START = 5
uint32 FLAG_GOAL = 6