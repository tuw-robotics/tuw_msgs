# This expresses a object in a 3D space
int32[] ids                  # possible ids of the detected object (empty if no id could be identified/assigned)
int32   shape                # shape type (possible types are: 
                             #             0: unknown
                             #             1: point, 2: circle, 3: ellipse, 4: square, 5: rectangle, 6: hull
                             #             102: sphere, 103: ellipsoid, 104: cube, 105: box, 106: mesh
float64[] shape_variables    # variables correspoding to the shape of the object 
                             # e.g. [] for point, [radius] for circle, [length,width] for rectangle
float64[] ids_confidence     # confidence of the possible ids in [0, 1]
geometry_msgs/Pose pose      # pose of the detected object
geometry_msgs/Twist twist    # twist of the detected object
