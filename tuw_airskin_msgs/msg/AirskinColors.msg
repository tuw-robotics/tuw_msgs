# This expresses a set of AirSkin pad colors
Header header                  # timestamp in the header is the acquisition time and frame

uint8[] ids                    # I2C ids of target AirSkin pads
std_msgs/ColorRGBA[] colors    # RGBA color values for corresponding AirSkin pads
