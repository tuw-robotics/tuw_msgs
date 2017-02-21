# This expresses a set of AirSkin pad pressures
Header header                  # timestamp in the header is the acquisition time and frame

uint8[] ids                    # I2C ids of target AirSkin pads
string[] names                 # Names of corresponding AirSkin pads
uint32[] pressures             # pressure values of corresponding AirSkin pads
