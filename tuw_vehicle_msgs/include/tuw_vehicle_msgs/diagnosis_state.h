#ifndef TUWR_MSGS_DIAGNOSIS_STATE_H
#define TUWR_MSGS_DIAGNOSIS_STATE_H

#include <tuwr_msgs/DiagnosisState.h>

namespace tuwr
{
namespace ros_msgs
{
class DiagnosisState : public tuwr_msgs::DiagnosisState
{
private:
  void SetOrClearError(bool error, uint8_t bit);

public:
  DiagnosisState();
  bool GetCanErrors();
  void SetCanErrors(bool error);
  bool GetInverterAliveError();
  void SetInverterAliveError(bool error);
  bool GetBmsVoltageError();
  void SetBmsVoltageError(bool error);
  bool HasErrors();
  void ClearErrors();
};
};
};
#endif // TUWR_MSGS_DIAGNOSIS_STATE_H