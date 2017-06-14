#include <tuw_vehicle_msgs/battery_state.h>
using namespace tuw::ros_msgs;

BatteryState::BatteryState(){

};

float BatteryState::GetLowestCellVoltage() {
  bool lowestFound = false;
  float lowest = 0;
  for (float const &voltage : this->lowestCellVoltages) {
    if (false == lowestFound || voltage < lowest) {
      lowest = voltage;
    }
  }
  return lowest;
}