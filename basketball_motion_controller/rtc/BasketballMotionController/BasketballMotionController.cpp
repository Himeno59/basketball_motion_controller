#include "BasketballMotionController.h"

#include <random>
#include <cmath>

BasketballMotionController::BasketballMotionController(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_objRefIn_("objRefIn", m_objRef_),
  
  // m_qRefIn_("qRefIn", m_qRef_),
  // m_tauRefIn_("tauRefIn", m_tauRef_),
  // m_qActIn_("qActIn", m_qAct_),
  // m_dqActIn_("dqActIn", m_dqAct_),
  // m_tauActIn_("tauActIn", m_tauAct_),
  // m_qOut_("q", m_q_),
  // m_tauOut_("tauOut", m_tau_),
  
  m_BasketballMotionControllerServicePort_("BasketballMotionControllerService"),
  m_AutoStabilizerServicePort_("AutoStabilizerService"),

  mt64(0),
  random(0.0, 0.05)
{
  this->m_service0_.setComp(this);

  motion_state = 0; // 0:停止時, 1:ff, 2:fb

  // [sec]
  dt = 0.002; 
  exec_tm = 0.0;

  epsilon = 1e-6;

  // state管理の都合上、dtの整数倍にしておく
  ff_motion_time = 0.35;
  fb_motion_time = 0.60; // 本当はfbの値から決める
  motion_time = ff_motion_time;

  // loop = 0;
  // max_count = static_cast<int>(motion_time/dt);
  // count = 0;

  // x, y, z : start, goal [m]
  // start, goalはドリブルをつく時を想定してある
  rarm_pos_range = {{0.6, 0.6}, {-0.4, -0.4}, {1.3, 1.1}};
  larm_pos_range = {{0.4, 0.4}, {0.4, 0.4}, {1.0, 1.0}};
  // r, p, y : start, goal [rad]
  rarm_rpy_range = {{-M_PI/2.0, -M_PI/2.0}, {-M_PI/10.0, M_PI/10.0}, {0.0, 0.0}};
  larm_rpy_range = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};

  startDribbleMode_flag = false;
  startDribbleMotion_flag = false;

  motionEnd_flag = false;
  last_motion = false;
  stateChange_flag = false;

  startBallPos.resize(3);
  goalBallPos.resize(3);
  bound_point.resize(3);
  dummyBallPos.resize(3);

}

RTC::ReturnCode_t BasketballMotionController::onInitialize(){
  addInPort("objRefIn", this->m_objRefIn_);

  // 各EndEffectorにつき、<name>PoseOutというOutPortをつくる  
  const std::vector<std::string> eeNames = {"rleg", "lleg", "rarm", "larm"};
  m_eePoseOut_.resize(eeNames.size());
  m_eePose_.resize(eeNames.size());
  for(size_t i=0;i<eeNames.size();i++){
    std::string portName = eeNames[i] + "PoseOut";
    m_eePoseOut_[i] = std::make_unique<RTC::OutPort<RTC::TimedPose3D> >(portName.c_str(), m_eePose_[i]);
    addOutPort(portName.c_str(), *(m_eePoseOut_[i]));
  }

  // targetEEPoseをresize
  targetEEPose.resize(eeNames.size());
  
  // addInPort("qRefIn", this->m_qRefIn_);
  // addInPort("tauRefIn", this->m_tauRefIn_);
  // addInPort("qActIn", this->m_qActIn_);
  // addInPort("dqActIn", this->m_dqActIn_);
  // addInPort("tauActIn", this->m_tauActIn_);
  // addOutPort("q", this->m_qOut_);
  // addOutPort("tauOut", this->m_tauOut_);
  m_BasketballMotionControllerServicePort_.registerProvider("service0", "BasketballMotionControllerService", m_service0_);
  addPort(m_BasketballMotionControllerServicePort_);
  m_AutoStabilizerServicePort_.registerConsumer("service0", "AutoStabilizerService", m_autoStabilizerService0_);
  addPort(m_AutoStabilizerServicePort_);
  
  return RTC::RTC_OK;
}

RTC::ReturnCode_t BasketballMotionController::onExecute(RTC::UniqueId ec_id){
  // std::cerr << "BasketballMotionController rtc onExecute" << std::endl;

  // if (this->m_qRefIn_.isNew()){
  //   this->m_qRefIn_.read();
  // }

  // if (this->m_tauRefIn_.isNew()){
  //   this->m_tauRefIn_.read();
  // }

  // if (this->m_qActIn_.isNew()){
  //   this->m_qActIn_.read();
  // }

  // if (this->m_dqActIn_.isNew()){
  //   this->m_dqActIn_.read();
  // }

  // if (this->m_tauActIn_.isNew()){
  //   this->m_tauActIn_.read();
  // }

  // {
  //   m_q_.tm = m_qRef_.tm;
  //   m_q_.data.length(m_qRef_.data.length());
  //   for (int i = 0 ; i < m_q_.data.length(); i++){
  //     m_q_.data[i] = m_qRef_.data[i];
  //   }
  //   this->m_qOut_.write();
  // }

  // {
  //   m_tau_.tm = m_qRef_.tm;
  //   m_tau_.data.length(m_tauRef_.data.length());
  //   for (int i = 0 ; i < m_q_.data.length(); i++){
  //     m_tau_.data[i] = m_tauRef_.data[i];
  //   }
  //   this->m_tauOut_.write();
  // }

  // startDribbleMode srv が呼ばれて、flag=trueになったら実行開始
  // 少し遅れてstartWholeBodyMasterSlave()を呼び、solveFKModeを切る
  if (!startDribbleMode_flag) return RTC::RTC_OK;

  // readInPortData
  if (m_objRefIn_.isNew()){
    m_objRefIn_.read();
  }

  // update param
  if (stateChange_flag) {
    updateParam();
    stateChange_flag = false; // とりあえずテストで切り替わったタイミングに一回paramを更新するのを試す
  }
  
  // eePoseのセット
  if (!startDribbleMotion_flag) {
    // startDribbleMotion srv が呼ばれて、flag=trueになるまではinitPoseを送り続ける
    initPose();
  } else {
    // generate targetEEPose
    genTargetEEPose();
  }
  
  // writeOutPortData
  writeOutPortData();

  return RTC::RTC_OK;
}

void BasketballMotionController::writeOutPortData() {
  // eePose
  for(int i=0;i<4;i++){
    // position
    m_eePose_[i].data.position.x = targetEEPose[i].data.position.x;
    m_eePose_[i].data.position.y = targetEEPose[i].data.position.y;
    m_eePose_[i].data.position.z = targetEEPose[i].data.position.z;
    // orientation
    m_eePose_[i].data.orientation.r = targetEEPose[i].data.orientation.r;
    m_eePose_[i].data.orientation.p = targetEEPose[i].data.orientation.p;
    m_eePose_[i].data.orientation.y = targetEEPose[i].data.orientation.y;
    
    m_eePoseOut_[i]->write();
  }
}

void BasketballMotionController::dummyObjPos() {
  dummyBallPos[0] = rarm_pos_range[0][0];
  dummyBallPos[1] = rarm_pos_range[1][0];
  dummyBallPos[2] = 1.3 + random(mt64);
  std::cout << "dummy_z:" << dummyBallPos[2] << std::endl;
}

void BasketballMotionController::updateParam() {
  // if (motion_state==1) {
  //     // fb_mode
  //     motion_state = 2;
  //     motion_time = fb_motion_time;
  //     // test
  //     dummyObjPos();
  //     rarm_pos_range[2][0] = dummyBallPos[2];
  //   } else if (motion_state==2) {
  //     // ff_mode
  //     motion_state = 1;
  //     motion_time = ff_motion_time;
  //   }
    
  // std::cout << "next-motion_state: " << motion_state << std::endl;
  
  if (motionEnd_flag) {
    // 動作停止モードへ以降
    if (motion_state==1) {
      motion_state = 2;
      motion_time = 2.0;
    } else if (motion_state==2) {
      motion_state = 1;
      motion_time = 2.0;
    }

    last_motion = true;
    
  } else {
    // ffとfbの切り替え
    if (motion_state==1) {
      // fb_mode
      motion_state = 2;
      motion_time = fb_motion_time;
      // test
      // dummyObjPos();
      // rarm_pos_range[2][0] = dummyBallPos[2];
    } else if (motion_state==2) {
      // ff_mode
      motion_state = 1;
      motion_time = ff_motion_time;
    }
    
    std::cout << "next-motion_state: " << motion_state << std::endl;
  }
}

void BasketballMotionController::genTargetEEPose() {
  if (motion_state==1) {
    ffTargetEEPose();
  } else if (motion_state==2) {
    fbTargetEEPose();
  } 
  
  exec_tm += dt;
  if (std::fabs(exec_tm - motion_time) < epsilon) {
    exec_tm = 0.0;
    stateChange_flag = true;

    if (motionEnd_flag && last_motion && (motion_state==2)) {
      startDribbleMotion_flag = false;
      motionEnd_flag = false;
      last_motion = false;
      
    }
  }
}

// todo: initPoseに遷移する関数を新たに作るか、ここの関数あたりに追加するか
void BasketballMotionController::ffTargetEEPose() {
  // rarm
  // position
  targetEEPose[2].data.position.x
    = ((rarm_pos_range[0][0] - rarm_pos_range[0][1]) / 2.0) * std::cos((M_PI/motion_time)*exec_tm)
    + ((rarm_pos_range[0][0] + rarm_pos_range[0][1]) / 2.0);; 
  targetEEPose[2].data.position.y
    = ((rarm_pos_range[1][0] - rarm_pos_range[1][1]) / 2.0) * std::cos((M_PI/motion_time)*exec_tm)
    + ((rarm_pos_range[1][0] + rarm_pos_range[1][1]) / 2.0);
  targetEEPose[2].data.position.z
    = ((rarm_pos_range[2][0] - rarm_pos_range[2][1]) / 2.0) * std::cos((M_PI/motion_time)*exec_tm)
    + ((rarm_pos_range[2][0] + rarm_pos_range[2][1]) / 2.0);
  // orientation
  targetEEPose[2].data.orientation.r
    = ((rarm_rpy_range[0][0] - rarm_rpy_range[0][1]) / 2.0) * std::cos((M_PI/motion_time)*exec_tm)
    + ((rarm_rpy_range[0][0] + rarm_rpy_range[0][1]) / 2.0);;
  targetEEPose[2].data.orientation.p
    = ((rarm_rpy_range[1][0] - rarm_rpy_range[1][1]) / 2.0) * std::cos((M_PI/motion_time)*exec_tm)
    + ((rarm_rpy_range[1][0] + rarm_rpy_range[1][1]) / 2.0);
  targetEEPose[2].data.orientation.y
    = ((rarm_rpy_range[2][0] - rarm_rpy_range[2][1]) / 2.0) * std::cos((M_PI/motion_time)*exec_tm)
    + ((rarm_rpy_range[2][0] + rarm_rpy_range[2][1]) / 2.0);;
  
  // larm
  targetEEPose[3].data.position.x = larm_pos_range[0][0];
  targetEEPose[3].data.position.y = larm_pos_range[1][0];
  targetEEPose[3].data.position.z = larm_pos_range[2][0];
  targetEEPose[3].data.orientation.r = larm_rpy_range[0][0];
  targetEEPose[3].data.orientation.p = larm_rpy_range[1][0];
  targetEEPose[3].data.orientation.y = larm_rpy_range[2][0];
  
  // legは全て0を送るので何もしない
}

void BasketballMotionController::fbTargetEEPose() {
  // モデル予測制御??
  
  // rarm
  // position
  targetEEPose[2].data.position.x
    = ((rarm_pos_range[0][0] - rarm_pos_range[0][1]) / 2.0) * std::cos((M_PI/motion_time)*exec_tm + M_PI)
    + ((rarm_pos_range[0][0] + rarm_pos_range[0][1]) / 2.0);; 
  targetEEPose[2].data.position.y
    = ((rarm_pos_range[1][0] - rarm_pos_range[1][1]) / 2.0) * std::cos((M_PI/motion_time)*exec_tm + M_PI)
    + ((rarm_pos_range[1][0] + rarm_pos_range[1][1]) / 2.0);
  targetEEPose[2].data.position.z
    = ((rarm_pos_range[2][0] - rarm_pos_range[2][1]) / 2.0) * std::cos((M_PI/motion_time)*exec_tm + M_PI)
    + ((rarm_pos_range[2][0] + rarm_pos_range[2][1]) / 2.0);
  // orientation
  targetEEPose[2].data.orientation.r
    = ((rarm_rpy_range[0][0] - rarm_rpy_range[0][1]) / 2.0) * std::cos((M_PI/motion_time)*exec_tm + M_PI)
    + ((rarm_rpy_range[0][0] + rarm_rpy_range[0][1]) / 2.0);;
  targetEEPose[2].data.orientation.p
    = ((rarm_rpy_range[1][0] - rarm_rpy_range[1][1]) / 2.0) * std::cos((M_PI/motion_time)*exec_tm + M_PI)
    + ((rarm_rpy_range[1][0] + rarm_rpy_range[1][1]) / 2.0);
  targetEEPose[2].data.orientation.y
    = ((rarm_rpy_range[2][0] - rarm_rpy_range[2][1]) / 2.0) * std::cos((M_PI/motion_time)*exec_tm + M_PI)
    + ((rarm_rpy_range[2][0] + rarm_rpy_range[2][1]) / 2.0);;
  
  // larm
  targetEEPose[3].data.position.x = larm_pos_range[0][0];
  targetEEPose[3].data.position.y = larm_pos_range[1][0];
  targetEEPose[3].data.position.z = larm_pos_range[2][0];
  targetEEPose[3].data.orientation.r = larm_rpy_range[0][0];
  targetEEPose[3].data.orientation.p = larm_rpy_range[1][0];
  targetEEPose[3].data.orientation.y = larm_rpy_range[2][0];
  
  // legは全て0を送るので何もしない
}
  
void BasketballMotionController::initPose(){
  // 単位は[m]
  // 右手
  targetEEPose[2].data.position.x = rarm_pos_range[0][0];
  targetEEPose[2].data.position.y = rarm_pos_range[1][0];
  targetEEPose[2].data.position.z = rarm_pos_range[2][0];
  targetEEPose[2].data.orientation.r = rarm_rpy_range[0][0];
  targetEEPose[2].data.orientation.p = rarm_rpy_range[1][0];
  targetEEPose[2].data.orientation.y = rarm_rpy_range[2][0];
  // 左手
  targetEEPose[3].data.position.x = larm_pos_range[0][0];
  targetEEPose[3].data.position.y = larm_pos_range[1][0];
  targetEEPose[3].data.position.z = larm_pos_range[2][0];
  targetEEPose[3].data.orientation.r = larm_rpy_range[0][0];
  targetEEPose[3].data.orientation.p = larm_rpy_range[1][0];
  targetEEPose[3].data.orientation.y = larm_rpy_range[2][0];
  // 脚は全て0を送るので何もしない
}

void BasketballMotionController::resetParam(){
  startDribbleMode_flag = false;
  startDribbleMotion_flag = false;
  motionEnd_flag = false;
  last_motion = false;

  motion_state = 1;
}


// srv
bool BasketballMotionController::startDribbleMode() {
  m_autoStabilizerService0_->getAutoStabilizerParam();
  
  startDribbleMode_flag = true;

  usleep(2000000); // 1秒以上待たないとmasterslaveが呼ばれない
  m_autoStabilizerService0_->startWholeBodyMasterSlave();
  
  return true;
}

bool BasketballMotionController::stopDribbleMode(){
  resetParam();
  exec_tm = 0.0;

  usleep(2000000); // 1秒以上待たないとmasterslaveが呼ばれない
  m_autoStabilizerService0_->stopWholeBodyMasterSlave();
  
  return true;
}

bool BasketballMotionController::startDribbleMotion(){
  if(!startDribbleMode_flag){
    std::cout << "Pleas Start DribbleMode" << std::endl;
    return false;
  } else {
    startDribbleMotion_flag = true;
    motion_state = 1;
    std::cout << "Start DribbleMode" << std::endl;
    return true;
  }
}

bool BasketballMotionController::stopDribbleMotion(){
  motionEnd_flag = true;
  // startDribbleMode_flag以外をfalse
  
  return true;
}

bool BasketballMotionController::BasketballMotionControllerParam(const double data){
  
}

static const char* BasketballMotionController_spec[] = {
  "implementation_id", "BasketballMotionController",
  "type_name",         "BasketballMotionController",
  "description",       "BasketballMotionController component",
  "version",           "0.0",
  "vendor",            "Tomoya-Himeno",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};


extern "C"{
  void BasketballMotionControllerInit(RTC::Manager* manager) {
    RTC::Properties profile(BasketballMotionController_spec);
    manager->registerFactory(profile, RTC::Create<BasketballMotionController>, RTC::Delete<BasketballMotionController>);
  }
};
