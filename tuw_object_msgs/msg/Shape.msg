
int64 id -1                                  # ID, -1 default
uint32 shape 0                               # Geometric shape, 0 default
uint32 type 0                                # Type, 0 default
geometry_msgs/Pose[]          poses          # poses describing the shape
tuw_std_msgs/ParameterArray[] params_poses   # parameters for each poses
tuw_std_msgs/ParameterArray   params         # parameters for the shape


uint32 SHAPE_NA = 0
uint32 SHAPE_POINT = 1
uint32 SHAPE_LINE = 2
uint32 SHAPE_LINE_STRIP = 3
uint32 SHAPE_CIRCLE = 4
uint32 SHAPE_POLYGON = 5
uint32 SHAPE_RECTANGLE = 6

uint32 TYPE_NA = 0

uint32 TYPE_MAP = 100            # map of shape rectangle, used to define a map frame

uint32 TYPE_PLANT = 200
uint32 TYPE_PLANT_WINE_ROW = 201

uint32 TYPE_OBSTACLE = 300
uint32 TYPE_OBSTACLE_HOUSE = 301
uint32 TYPE_OBSTACLE_TREE = 302

uint32 TYPE_TRANSIT = 400
uint32 TYPE_TRANSIT_STREET = 401
uint32 TYPE_TRANSIT_GRAVEL = 402

uint32 TYPE_WALL = 500
uint32 TYPE_WALL_ROOM = 501
uint32 TYPE_WALL_HALLWAY = 502
