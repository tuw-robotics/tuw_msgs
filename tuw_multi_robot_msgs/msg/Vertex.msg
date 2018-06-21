#################################################################
## A single vertex in a graph 
## Each vertex of the same graph must have a unique id. 
## Successors and Predecessors must have a common start or end
## point 
#################################################################
uint32 id                   # Vertex id
bool valid                  # true if it can be for planning
geometry_msgs/Point[] path  # points describing the vertex start and endpoint
                            #    the first point in the array reprecents the start and the last the endpoint 
uint32 weight               # the weight of the vertex (e.g. length of the segment)
float32 width               # fee space next to the vertex
uint32[] successors         # edges to successors
uint32[] predecessors       # edges to predecessor
 
