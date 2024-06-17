#ifndef BasketballMotionController_H
#define BasketballMotionController_H

#include <memory>
#include <cmath>
#include <random>
#include <chrono>

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
#include "PolynomialInterpolator.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define deg2rad(x) ((x) * M_PI / 180.0)

class BasketballMotionController : public RTC::DataFlowComponentBase{
protected:
    
  // InPort
  // ボールの予測軌道、速度
  basketball_motion_controller_msgs::ObjStateStamped m_nowObjState_;
  RTC::InPort<basketball_motion_controller_msgs::ObjStateStamped> m_nowObjStateIn_;
  basketball_motion_controller_msgs::ObjStateStamped m_predObjState_;
  RTC::InPort<basketball_motion_controller_msgs::ObjStateStamped> m_predObjStateIn_;
  // fbを始めるかどうかのflag
  RTC::TimedBoolean m_fbFlag_;
  RTC::InPort<RTC::TimedBoolean> m_fbFlagIn_;

  // OutPort
  // refEEPose
  std::vector<RTC::TimedPose3D> m_eePose_;
  std::vector<std::unique_ptr<RTC::OutPort<RTC::TimedPose3D>>> m_eePoseOut_;

  BasketballMotionControllerService_impl m_service0_;
  RTC::CorbaPort m_BasketballMotionControllerServicePort_;

  // SolveFKModeをoffにするためにWholeBodyMasterSlaveを利用
  RTC::CorbaConsumer<OpenHRP::AutoStabilizerService> m_autoStabilizerService0_;
  RTC::CorbaPort m_AutoStabilizerServicePort_;

protected:
  // utility functions
  void readInPortData();
  void writeOutPortData();
  void updateParam();
  void stateManager();
  void genTargetEEPose();
  void ffTargetEEPose();
  void fbTargetEEPose();

  void calcEEVelocity();
  void setPolynomialConditions();

  void initPose();
  void resetParam();
  void dummyObjPos();
  bool getProperty(const std::string& key, std::string& ret); // dtをconfファイルからloadする用
  
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
  bool setBasketballMotionControllerParam(const double data);
  bool getBasketballMotionControllerParam(const double data);

private:
  // 時間の管理
  double dt;      // 0.0020 <- confファイルからloadしてくる
  double exec_tm; // motionを開始してからの経過時間(実行時間)
  double epsilon; // doubleの比較判定に使用

  // motion_time = ff_motion_time or fb_motion_time
  double motion_time;
  double ff_motion_time;
  double fb_motion_time;
  
  // ドリブルの状態を区切る
  int motion_state; // 0:動作前, 1:ff, 2:fb

  // ボールの半径
  double ball_r;

  // 目標の接触時ボール状態(pos,vel)
  // この変数を元に他のパラメタを決定する
  // 今は目標の最高到達点の位置を設定しておく
  // 0:右手, 1:左手
  std::vector<basketball_motion_controller_msgs::ObjStateStamped> targetContactBallState;

  // 予測した接触時のボールの状態(pos,vel)
  // 今はpredObjeStateをそのまま代入し、最高到達点を次の接触状態とする
  // このボールを右手でつくのか左手でつくのかを決めるフラグが必要になりそう
  basketball_motion_controller_msgs::ObjStateStamped nextContactBallState;

  // 目標の接触時ハンド状態
  // 0:右手, 1:左手
  // 必要な情報: x,y,zとその1階微分,2階微分, r,p,yとその1階微分,2階微分 -> 計18個
  // {{x, dot(x), dot(dot(x))}, {y, dot(y), dot(dot(y))}, {z, dot(z), dot(dot(z))}, ~}
  // x,y,z,r,p,y
  std::vector<std::vector<double>> targetContactEEPos;
  std::vector<std::vector<double>> targetContactEERPy
  
  // ff
  /* std::vector<RTC::TimedPose3D> targetEEPose; */
  /* std::vector<RTC::TimedPose3D> prevEEPose; */
  std::vector<std::vector<double>> targetEEPose;
  std::vector<std::vector<double>> prevEEPose;
  std::vector<std::vector<double>> EEVel;
  std::vector<std::vector<double>> prevEEvel;
  std::vector<std::vector<double>> EEAcc;
  
  // std::vector<RTC::~~> targetEEState; // pos,vel,orientation,angular-vel
  std::vector<std::vector<double>> rarm_pos_range;
  std::vector<std::vector<double>> larm_pos_range;
  std::vector<std::vector<double>> rarm_rpy_range;
  std::vector<std::vector<double>> larm_rpy_range;

  std::vector<std::vector<double>> rarm_pos;
  std::vector<std::vector<double>> rarm_vel;
  std::vector<std::vector<double>> rarm_acc;
  std::vector<std::vector<double>> rarm_acc;

  std::vector<double> bound_point;   // 地面への衝突位置(z=0)
  std::vector<double> d;             // bound_point計算用
  std::vector<double> dummyBallPos;  // test
  
  bool startDribbleMode_flag;
  bool startDribbleMotion_flag;
  
  bool motionEnd_flag;
  bool last_motion;
  bool stateChange_flag;
  bool loop_init;

  std::mt19937_64 mt64;
  std::uniform_real_distribution<double> random;

  // fb
  PolynomialInterpolator fbInterpolator;
  double tf;
  std::vector<BoundaryConditions> pos_start_conditions;
  std::vector<BoundaryConditions> pos_end_conditions;
  std::vector<BoundaryConditions> rpy_start_conditions;
  std::vector<BoundaryConditions> rpy_end_conditions;
  Eigen::VectorXd fbTrajectoryTheta;

};

extern "C"
{
  void BasketballMotionControllerInit(RTC::Manager* manager);
}

#endif // BasketballMotionController_H
