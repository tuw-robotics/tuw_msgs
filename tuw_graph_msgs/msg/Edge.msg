#################################################################
## A single edge in a graph
## Each edge of the same graph must have a unique id.
#################################################################
int64 id                   # Edge id
bool valid                  # True if it can be used for planning
float64 weight              # Weight of the edge
uint32[] flags               # Flags or attributes
int64 start                 # Ids start node
int64 end                   # Ids end node
geometry_msgs/Pose[] path   # Path between the nodes
                            # - path can be empty 
                            # - path does does not include the nodes locations




uint32 FLAG_NA = 0
uint32 FLAG_UNTOUCHED = 1
uint32 FLAG_OPEN = 2
uint32 FLAG_CLOSED = 3
uint32 FLAG_VISITED = 4