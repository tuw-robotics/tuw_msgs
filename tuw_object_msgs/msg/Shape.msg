
int64 id                                     # ID
uint32 shape                                 # Geometric shape
uint32 type                                  # Type
geographic_msgs/GeoPoint[] wgs84             # global location in WGS84
geometry_msgs/Point[]      points            # map location in cartesian space
tuw_std_msgs/ParameterArray[] params_points  # parameters for each point
tuw_std_msgs/ParameterArray   params         # parameters for the shape


uint32 SHAPE_NA = 0
uint32 SHAPE_POINT = 1
uint32 SHAPE_LINE = 2
uint32 SHAPE_LINE_STRIP = 3

uint32 TYPE_NA = 0

uint32 TYPE_FRAME_ORIGIN = 100   # frame origin
uint32 TYPE_FRAME_WINDOW = 101   # frame window size, two points top left bottom right

uint32 TYPE_PLANT = 200
uint32 TYPE_PLANT_WINE_ROW = 201

uint32 TYPE_OBSTACLE = 300
uint32 TYPE_OBSTACLE_HOUSE = 301
uint32 TYPE_OBSTACLE_TREE = 302

uint32 TYPE_TRANSIT = 400
uint32 TYPE_TRANSIT_STREET = 401
uint32 TYPE_TRANSIT_GRAVEL = 402
