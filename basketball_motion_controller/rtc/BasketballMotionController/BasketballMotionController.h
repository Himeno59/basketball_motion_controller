#ifndef BasketballMotionController_H
#define BasketballMotionController_H

#include <memory>
#include <cmath>
#include <random>
#include <chrono>
#include <mutex>
#include <eigen3/Eigen/Dense>

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
  // tmを同期させる用
  RTC::TimedDoubleSeq m_qRef_;
  RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn_;
  
  // OutPort
  // refEEPose
  std::vector<RTC::TimedPose3D> m_eePose_;
  std::vector<std::unique_ptr<RTC::OutPort<RTC::TimedPose3D>>> m_eePoseOut_;

  // log用
  std::vector<RTC::TimedPoint3D> m_eePoint_;
  std::vector<std::unique_ptr<RTC::OutPort<RTC::TimedPoint3D>>> m_eePointOut_;
  std::vector<RTC::TimedOrientation3D> m_eeOrientation_;
  std::vector<std::unique_ptr<RTC::OutPort<RTC::TimedOrientation3D>>> m_eeOrientationOut_;
  
  BasketballMotionControllerService_impl m_service0_;
  RTC::CorbaPort m_BasketballMotionControllerServicePort_;

  // SolveFKModeをoffにするためにWholeBodyMasterSlaveを利用
  RTC::CorbaConsumer<OpenHRP::AutoStabilizerService> m_autoStabilizerService0_;
  RTC::CorbaPort m_AutoStabilizerServicePort_;

protected:
  // utility functions
  void readInPortData();
  void writeOutPortData();
  void updateParams();
  void stateManager();
  void genTargetEEPose();
  void ffTargetEEPose();
  void fbTargetEEPose();
  void setFeedForwardParams();
  void setPolynomialConditions();
  void calcContactEEPose();

  void initPose();
  void resetParam();
  void dummyObjPos();
  bool getProperty(const std::string& key, std::string& ret); // dtをconfファイルからloadする用

  std::mutex mutex_;
  
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
  
  bool setBasketballMotionControllerParam(const OpenHRP::BasketballMotionControllerService::BasketballMotionControllerParam& i_param);
  bool getBasketballMotionControllerParam(OpenHRP::BasketballMotionControllerService::BasketballMotionControllerParam& i_param);

private:
  // 時間の管理
  double dt;         // 0.0020 <- confファイルからloadしてくる
  double exec_tm;    // motionを開始してからの経過時間(実行時間)
  double epsilon;    // doubleの比較判定に使用
  double contact_tm; // フィードバックのフラグが立ってから最高到達点に達するまでの時間

  double motion_time; // フィードフォワード、フィードバックで切り替える
  double ff_motion_time;
  
  // ドリブルの状態を区切る
  int motion_state; // 0:動作前, 1:ff, 2:fb

  // ball->handのためのオフセット
  // x,y,z,r,p,y
  std::vector<double> hand_offset;
  
  // 目標の接触時ボール状態(pos,vel)
  // ドリブルしたボールがこの位置に跳ね返ってくるようにドリブルの軌道を生成する
  // 今は目標の最高到達点の位置を設定しておく
  // 0:右手, 1:左手
  std::vector<basketball_motion_controller_msgs::ObjStateStamped> targetContactBallState;

  // ボールが遠くに行ったときに危険な動作をしないための制限
  // targetContactBallStateからの距離
  double maxDribbleReach;
 
  // ドリブル1サイクルにかける時間
  // これは上のvelから決まってしまう??
  
  // 予測した接触時ボール状態(pos,vel)
  // targetContactBallStateにボールが来ることが期待されるが、実際はそこに来ないのでビジョンfbによる次の接触タイミングを設定する
  // 今はpredObjeStateをそのまま代入し、最高到達点を次の接触状態とする
  // このボールを右手でつくのか左手でつくのかを決めるフラグが必要になりそう
  basketball_motion_controller_msgs::ObjStateStamped nextContactBallState;

  // 目標の接触時ハンド状態
  // row: 0~3 -> rleg, lleg, rarm, larm
  // x,y,z,r,p,y
  Eigen::MatrixXd targetContactEEPose;
  Eigen::MatrixXd targetContactEEVel;
  Eigen::MatrixXd targetContactEEAcc;

  // 計算に使用するハンドのpose,vel,acc
  // row: 0~3 -> rleg, lleg, rarm, larm
  // columｎ0~5: x, y, z, r, p, y
  Eigen::MatrixXd targetEEPose;
  Eigen::MatrixXd prevEEPose;
  Eigen::MatrixXd EEVel;
  Eigen::MatrixXd prevEEVel;
  Eigen::MatrixXd EEAcc;

  // 三角関数ff軌道のパラメタ
  // {start, range}
  std::vector<std::vector<double>> rarm_pos_params;
  std::vector<std::vector<double>> larm_pos_params;
  std::vector<std::vector<double>> rarm_rpy_params;
  std::vector<std::vector<double>> larm_rpy_params;

  // 将来的に使うかも??
  std::vector<double> bound_point;   // 地面への衝突位置(z=0)
  std::vector<double> d;             // bound_point計算用
  std::vector<double> dummyBallPos;  // test

  // flag
  bool startDribbleMode_flag;    // dribble-modeの姿勢 = initPoseの姿勢への遷移
  bool startDribbleMotion_flag;  // motionのスタート
  
  bool motionEnd_flag;           // stop-dribble-motionが呼ばれたかどうか
  bool last_motion;              // 最後のmotionかどうか
  bool stateChange_flag;         // ffとfbの切り替わりが有るかどうか
  bool loop_init;                // 各loopの最初が0.0秒になるようにする
  bool fb_initialize;            // fbが入った最初のタイミングで多項式の係数計算をする
  bool fb_flag;                  // object_trajectory_estimatorからfbのフラグが来たかどうか

  // test用
  bool test_flag;
  std::mt19937_64 mt64;
  std::uniform_real_distribution<double> random1;
  std::uniform_real_distribution<double> random2;

  // fbの多項式補間軌道の係数
  PolynomialInterpolator fbInterpolator;
  int polynomial_degree;
  std::vector<Eigen::VectorXd> rarm_fb_theta; // 6 * (polynomial_degree + 1)
  std::vector<Eigen::VectorXd> larm_fb_theta; 
  std::vector<BoundaryConditions> start_conditions; // 6 * (polynomial_degree + 1)
  std::vector<BoundaryConditions> end_conditions;
  

};

extern "C"
{
  void BasketballMotionControllerInit(RTC::Manager* manager);
}

#endif // BasketballMotionController_H
