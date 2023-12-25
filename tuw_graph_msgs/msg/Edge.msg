#################################################################
## A single edge in a graph
## Each edge of the same graph must have a unique id.
#################################################################
int64 id                   # Edge id
bool valid                  # True if it can be used for planning
bool directed               # True if it is a directed edges
float64 weight              # Weight of the edge
int32[] flags               # Flags or attributes
int64 start                 # Ids start node
int64 end                   # Ids end node
geometry_msgs/Pose[] path   # Path between the nodes
                            # - path can be empty 
                            # - path does does not include the nodes locations




int32 FLAG_ONCE = 0                  # can only used once
int32 FLAG_PRECONDITION = 1          # check for pre conditions
int32 FLAG_POSTCONDITION = 2         # check for post conditions