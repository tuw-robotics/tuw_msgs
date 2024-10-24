
int64 id                         # ID
uint32 type                      # Type
ObjectPoint[] points                   # Points that describe the object
tuw_std_msgs/ParameterFloat64[] parameters               # Parameters that describe the object

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
