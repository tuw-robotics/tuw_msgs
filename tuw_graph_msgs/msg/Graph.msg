#################################################################
## This message is used to transport a graph between two nodes.
## Each vertex has a set of points and a width describing the
## shape of it and additionally successors and predecessors
## to describe the overall structure of the graph
##
## Remark for the TUW_Multi_Robot_Router
## !! all vertices have to be sorted by id
## !! every id must be present...
#################################################################

std_msgs/Header header      # Header for Timestamp and Id
geometry_msgs/Pose origin   # Graph origin
Node[] nodes                # The nodes in the graph
Edge[] edges                # The edges in the graph