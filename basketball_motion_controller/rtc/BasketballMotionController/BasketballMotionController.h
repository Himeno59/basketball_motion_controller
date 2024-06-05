#ifndef BasketballMotionController_H
#define BasketballMotionController_H

#include <memory>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>
#include <rtm/CorbaPort.h>
#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>

#include <basketball_motion_controller_msgs/idl/BasketballMotionController.hh>

#include "BasketballMotionControllerService_impl.h"

class BasketballMotionController : public RTC::DataFlowComponentBase{
protected:
    
  // inport
  // ボールの予測軌道、速度
  RTC::TimedDoubleSeq m_objRef_;
  RTC::InPort<RTC::TimedDoubleSeq> m_objRefIn_;
  
  /* RTC::TimedDoubleSeq m_qRef_; */
  /* RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn_; */
  /* RTC::TimedDoubleSeq m_tauRef_; */
  /* RTC::InPort<RTC::TimedDoubleSeq> m_tauRefIn_; */
  /* RTC::TimedDoubleSeq m_qAct_; */
  /* RTC::InPort<RTC::TimedDoubleSeq> m_qActIn_; */
  /* RTC::TimedDoubleSeq m_dqAct_; */
  /* RTC::InPort<RTC::TimedDoubleSeq> m_dqActIn_; */
  /* RTC::TimedDoubleSeq m_tauAct_; */
  /* RTC::InPort<RTC::TimedDoubleSeq> m_tauActIn_; */

  // outport
  // refEEPose
  // 時系列のデータにする必要はある？
  std::vector<RTC::TimedPose3D> m_eePose_;
  std::vector<std::unique_ptr<RTC::OutPort<RTC::TimedPose3D> > > m_eePoseOut_;
  
  /* RTC::TimedDoubleSeq m_q_; */
  /* RTC::OutPort<RTC::TimedDoubleSeq> m_qOut_; */
  /* RTC::TimedDoubleSeq m_tau_; */
  /* RTC::OutPort<RTC::TimedDoubleSeq> m_tauOut_; */
  /* RTC::TimedDoubleSeq m_ */

  BasketballMotionControllerService_impl m_service0_;
  RTC::CorbaPort m_basketballmotionControllerServicePort_;

protected:
  void genTargetEEPose();
  
public:
  BasketballMotionController(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool basketballmotionParam(const double data);

private:
  int loop; // ドリブルのカウント
  double dt;
  double exec_tm;

  double motion_time;
  std::vector<std::vector<double>> pos_range;
  std::vector<std::vector<double>> rpy_range;
  
};

extern "C"
{
  void BasketballMotionControllerInit(RTC::Manager* manager);
}

#endif // BasketballMotionController_H
