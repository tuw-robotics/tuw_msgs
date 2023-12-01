#################################################################
## A single edge in a graph
## Each edge of the same graph must have a unique id.
#################################################################
int64 id                   # Edge id
bool valid                  # True if it can be used for planning
bool directed               # True if it is a directed edges
float64 weight              # Weight of the edge
int64[2] nodes               # Ids of the nodes connected with this edge
                            # start node, end node
geometry_msgs/Pose[] path   # Path between the nodes
                            # - path can be empty 
                            # - path does does not include the nodes locations