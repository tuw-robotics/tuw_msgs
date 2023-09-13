#################################################################
## A single edge in a graph
## Each edge of the same graph must have a unique id.
#################################################################
uint32 id                   # Edge id
bool valid                  # True if it can be used for planning
bool directed               # True if it is a directed edges
float32 weight              # Weight of the edge
uint32[] nodes              # Ids of the nodes connected with this edge
                            # size of two
                            # start node, end node
geometry_msgs/Point[] path  # Path between the nodes
                            # - path can be empty 
                            # - path does does not include the nodes locations