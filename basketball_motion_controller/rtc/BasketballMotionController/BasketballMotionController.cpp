#include "BasketballMotionController.h"

#include <random>
#include <cmath>

BasketballMotionController::BasketballMotionController(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_objRefIn_("objRefIn", m_objRef_),
  
  m_BasketballMotionControllerServicePort_("BasketballMotionControllerService"),
  m_AutoStabilizerServicePort_("AutoStabilizerService"),

  mt64(0),
  random(0.0, 0.05)
{
  this->m_service0_.setComp(this);

  motion_state = 0; // 0:停止時, 1:ff, 2:fb
  
  exec_tm = 0.0;
  epsilon = 1e-6;

  // state管理の都合上、dtの整数倍にしておく
  ff_motion_time = 0.30;
  fb_motion_time = 0.45; // 本当はfbの結果から
  motion_time = ff_motion_time;

  // ffのmotionのパラメタ
  // x, y, z : start, goal [m]
  // start, goalはドリブルをつく時を想定してある
  rarm_pos_range = {{0.60, 0.62}, {-0.45, -0.45}, {1.3, 1.0}};
  // rarm_pos_range = {{0.70, 0.72}, {-0.40, -0.40}, {1.3, 1.0}};
  larm_pos_range = {{0.0, 0.0}, {0.45, 0.45}, {0.68, 0.68}};
  // r, p, y : start, goal [rad]
  rarm_rpy_range = {{-M_PI/2.0, -M_PI/2.0}, {-M_PI/8.0, M_PI/8.0}, {0.0, 0.0}};
  // rarm_rpy_range = {{-M_PI/2.0, -M_PI/2.0}, {-M_PI/20.0, M_PI/20.0}, {0.0, 0.0}};
  larm_rpy_range = {{0.0, 0.0}, {M_PI/2.5, M_PI/2.5}, {0.0, 0.0}};

  // 初期姿勢の位置でとりあえず初期化しておく
  targetContactBallState = {{rarm_pos_range[0][0], rarm_pos_range[1][0], rarm_pos_range[2][0]},
			    {0.0, 0.0, 0.0}}
  
  // resize
  startBallPos.resize(3);
  goalBallPos.resize(3);
  bound_point.resize(3);
  dummyBallPos.resize(3);
  
  
  // flag
  motionEnd_flag = false;
  last_motion = false;
  stateChange_flag = false;

  startDribbleMode_flag = false;
  startDribbleMotion_flag = false;
}

RTC::ReturnCode_t BasketballMotionController::onInitialize(){
  // InPort
  addInPort("objRefIn", this->m_objRefIn_);

  // OutPort
  // 各EndEffectorにつき、<name>PoseOutというOutPortをつくる  
  const std::vector<std::string> eeNames = {"rleg", "lleg", "rarm", "larm"};
  m_eePoseOut_.resize(eeNames.size());
  m_eePose_.resize(eeNames.size());
  for(size_t i=0;i<eeNames.size();i++){
    std::string portName = eeNames[i] + "PoseOut";
    m_eePoseOut_[i] = std::make_unique<RTC::OutPort<RTC::TimedPose3D> >(portName.c_str(), m_eePose_[i]);
    addOutPort(portName.c_str(), *(m_eePoseOut_[i]));
  }

  // resize
  targetEEPose.resize(eeNames.size());
  targetContactEEPose.resize(eeNames.size());
  
  // srv
  m_BasketballMotionControllerServicePort_.registerProvider("service0", "BasketballMotionControllerService", m_service0_);
  addPort(m_BasketballMotionControllerServicePort_);
  m_AutoStabilizerServicePort_.registerConsumer("service0", "AutoStabilizerService", m_autoStabilizerService0_);
  addPort(m_AutoStabilizerServicePort_);

  {
    // load dt
    std::string buf; getProperty("dt", buf);
    dt = std::stod(buf); // str->double
    if(dt <= 0.0){
      getProperty("exec_cxt.periodic.rate", buf);
      double rate = std::stod(buf);
      if(rate > 0.0){
        dt = 1.0/rate;
      }else{
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "dt is invalid" << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
    }
  }
  
  return RTC::RTC_OK;
}

RTC::ReturnCode_t BasketballMotionController::onExecute(RTC::UniqueId ec_id){
  // startDribbleModeが呼ばれて、flag=trueになったら実行開始
  if (!startDribbleMode_flag) return RTC::RTC_OK;

  // readInPortData
  readInPortData();

  // update param
  if (stateChange_flag) {
    updateParam();
    stateChange_flag = false; // とりあえずテストで切り替わったタイミングに一回paramを更新するのを試す
  }
  
  // eePoseの計算
  if (!startDribbleMotion_flag) {
    // startDribbleMotionが呼ばれてflag=trueになるまではinitPoseを送り続ける
    initPose();
  } else {
    genTargetEEPose();
  }
  
  // writeOutPortData
  writeOutPortData();
  
  return RTC::RTC_OK;
}

void BasketballMotionController::readInPortData() {
  // Objectoの予測軌道
  // 一旦最高到達点が流れてくると仮定する
  if (m_objRefIn_.isNew()){
    m_objRefIn_.read();
  }
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
      dummyObjPos();
      rarm_pos_range[2][0] = dummyBallPos[2];
    } else if (motion_state==2) {
      // ff_mode
      motion_state = 1;
      motion_time = ff_motion_time;
      
      // rangeの設定
      // startは最高到達点でcontactしたところ
      // goalはどうする??とりあえず変更なし??
      rarm_pos_range[0][0] = targetContactEEPose[0];
      rarm_pos_range[1][0] = targetContactEEPose[1];
      rarm_pos_range[2][0] = targetContactEEPose[2];
    }
  }
  
}

void BasketballMotionController::genTargetEEPose() {
  // ffとfbの切り替え
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
    = ((rarm_pos_range[0][0] - rarm_pos_range[0][1]) / 2.0) * std::cos((2*M_PI/motion_time)*exec_tm)
    + ((rarm_pos_range[0][0] + rarm_pos_range[0][1]) / 2.0);; 
  targetEEPose[2].data.position.y
    = ((rarm_pos_range[1][0] - rarm_pos_range[1][1]) / 2.0) * std::cos((2*M_PI/motion_time)*exec_tm)
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
  // rarm
  // position
  targetEEPose[2].data.position.x
    = ((rarm_pos_range[0][0] - rarm_pos_range[0][1]) / 2.0) * std::cos((2*M_PI/motion_time)*exec_tm)
    + ((rarm_pos_range[0][0] + rarm_pos_range[0][1]) / 2.0);; 
  targetEEPose[2].data.position.y
    = ((rarm_pos_range[1][0] - rarm_pos_range[1][1]) / 2.0) * std::cos((2*M_PI/motion_time)*exec_tm)
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
  
  // // rarm
  // // position
  // targetEEPose[2].data.position.x
  //   = ((rarm_pos_range[0][0] - rarm_pos_range[0][1]) / 2.0) * std::cos((2*M_PI/motion_time)*exec_tm)
  //   + ((rarm_pos_range[0][0] + rarm_pos_range[0][1]) / 2.0);; 
  // targetEEPose[2].data.position.y
  //   = ((rarm_pos_range[1][0] - rarm_pos_range[1][1]) / 2.0) * std::cos((2*M_PI/motion_time)*exec_tm)
  //   + ((rarm_pos_range[1][0] + rarm_pos_range[1][1]) / 2.0);
  // targetEEPose[2].data.position.z
  //   = ((rarm_pos_range[2][0] - rarm_pos_range[2][1]) / 2.0) * std::cos((M_PI/motion_time)*exec_tm + M_PI)
  //   + ((rarm_pos_range[2][0] + rarm_pos_range[2][1]) / 2.0);
  // // orientation
  // targetEEPose[2].data.orientation.r
  //   = ((rarm_rpy_range[0][0] - rarm_rpy_range[0][1]) / 2.0) * std::cos((M_PI/motion_time)*exec_tm + M_PI)
  //   + ((rarm_rpy_range[0][0] + rarm_rpy_range[0][1]) / 2.0);;
  // targetEEPose[2].data.orientation.p
  //   = ((rarm_rpy_range[1][0] - rarm_rpy_range[1][1]) / 2.0) * std::cos((M_PI/motion_time)*exec_tm + M_PI)
  //   + ((rarm_rpy_range[1][0] + rarm_rpy_range[1][1]) / 2.0);
  // targetEEPose[2].data.orientation.y
  //   = ((rarm_rpy_range[2][0] - rarm_rpy_range[2][1]) / 2.0) * std::cos((M_PI/motion_time)*exec_tm + M_PI)
  //   + ((rarm_rpy_range[2][0] + rarm_rpy_range[2][1]) / 2.0);;
  
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

bool BasketballMotionController::getProperty(const std::string& key, std::string& ret) {
  if (this->getProperties().hasKey(key.c_str())) {
    ret = std::string(this->getProperties()[key.c_str()]);
  } else if (this->m_pManager->getConfig().hasKey(key.c_str())) { // 引数 -o で与えたプロパティを捕捉
    ret = std::string(this->m_pManager->getConfig()[key.c_str()]);
  } else {
    return false;
  }
  std::cerr << "[" << this->m_profile.instance_name << "] " << key << ": " << ret <<std::endl;
  return true;
}

// srv
bool BasketballMotionController::startDribbleMode() {
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
    std::cout << "Please Start DribbleMode" << std::endl;
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
