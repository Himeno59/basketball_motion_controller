#ifndef BasketballMotionController_H
#define BasketballMotionController_H

#include <memory>
#include <cmath>
#include <random>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>
#include <rtm/CorbaPort.h>
#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <cnoid/EigenUtil>

#include <basketball_motion_controller_msgs/idl/BasketballMotionController.hh>
#include <auto_stabilizer/idl/AutoStabilizerService.hh>

#include "BasketballMotionControllerService_impl.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define deg2rad(x) ((x) * M_PI / 180.0)

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
  RTC::CorbaPort m_BasketballMotionControllerServicePort_;
  
  RTC::CorbaConsumer<OpenHRP::AutoStabilizerService> m_autoStabilizerService0_;
  RTC::CorbaPort m_AutoStabilizerServicePort_;

protected:
  // utility functions
  // void readInPortData();
  void writeOutPortData();
  void updateParam();
  void genTargetEEPose();
  void ffTargetEEPose();
  void fbTargetEEPose();

  void initPose();
  void resetParam();
  void dummyObjPos();
  
public:
  BasketballMotionController(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  // virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // srv
  bool startDribbleMode();
  bool stopDribbleMode();
  bool startDribbleMotion();
  bool stopDribbleMotion();

  bool BasketballMotionControllerParam(const double data);

private:

  double dt;
  double exec_tm;

  double epsilon;
  
  double motion_time; // ff_motion_time or fb_motion_time
  double ff_motion_time;
  double fb_motion_time;
  
  int loop; // ドリブルのサイクルの切れ目をどうするか
  int max_count; // 1サイクルの中でdt何個分か
  int count; // dt何個目か

  // ドリブルの状態を区切る
  // 0:動作前, 1:振り下ろし, 2:fb
  int motion_state; 

  // 0:rarm, 1:larm
  std::vector<RTC::TimedPose3D> targetEEPose;
  // std::vector<cnoid::Position> targetEEPose; // cnoid::Positionで扱ったほうが良かったりする??
  std::vector<std::vector<double>> rarm_pos_range;
  std::vector<std::vector<double>> larm_pos_range;
  std::vector<std::vector<double>> rarm_rpy_range;
  std::vector<std::vector<double>> larm_rpy_range;

  std::vector<double> startBallPos;  // (最高到達点に達した)今のボールの位置
  std::vector<double> goalBallPos;   // バウンドした後のボールの最高到達位置
  std::vector<double> bound_point;   // 地面への衝突位置(z=0)
  std::vector<double> d;             // bound_point計算用
  std::vector<double> dummyBallPos;  // test
  
  bool startDribbleMode_flag;
  bool startDribbleMotion_flag;
  
  bool motionEnd_flag;
  bool last_motion;
  bool stateChange_flag;

  std::mt19937_64 mt64;
  std::uniform_real_distribution<double> random;

};

extern "C"
{
  void BasketballMotionControllerInit(RTC::Manager* manager);
}

#endif // BasketballMotionController_H
