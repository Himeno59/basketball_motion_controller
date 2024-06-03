#ifndef BasketballMotionController_H
#define BasketballMotionController_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>
#include <rtm/CorbaPort.h>
#include <rtm/idl/BasicDataType.hh>

#include <basketball_motion_controller_msgs/idl/BasketballMotionController.hh>

#include "BasketballMotionControllerService_impl.h"

class BasketballMotionController : public RTC::DataFlowComponentBase{
protected:

  RTC::TimedDoubleSeq m_qRef_;
  RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn_;
  RTC::TimedDoubleSeq m_tauRef_;
  RTC::InPort<RTC::TimedDoubleSeq> m_tauRefIn_;
  RTC::TimedDoubleSeq m_qAct_;
  RTC::InPort<RTC::TimedDoubleSeq> m_qActIn_;
  RTC::TimedDoubleSeq m_dqAct_;
  RTC::InPort<RTC::TimedDoubleSeq> m_dqActIn_;
  RTC::TimedDoubleSeq m_tauAct_;
  RTC::InPort<RTC::TimedDoubleSeq> m_tauActIn_;
  RTC::TimedDoubleSeq m_q_;
  RTC::OutPort<RTC::TimedDoubleSeq> m_qOut_;
  RTC::TimedDoubleSeq m_tau_;
  RTC::OutPort<RTC::TimedDoubleSeq> m_tauOut_;

  BasketballMotionControllerService_impl m_service0_;
  RTC::CorbaPort m_basketballmotionControllerServicePort_;

public:
  BasketballMotionController(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool basketballmotionParam(const double data);

private:
};

extern "C"
{
  void BasketballMotionControllerInit(RTC::Manager* manager);
}

#endif // BasketballMotionController_H
