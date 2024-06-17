#oinclude "BasketballMotionController.h"

#include <random>
#include <cmath>

BasketballMotionController::BasketballMotionController(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_nowObjStateIn_("nowObjStateIn", m_nowObjState_),
  m_predObjStateIn_("predObjStateIn", m_predObjState_),
  m_fbFlagIn_("fbFlagIn", m_fbFlag_),
  
  m_BasketballMotionControllerServicePort_("BasketballMotionControllerService"),
  m_AutoStabilizerServicePort_("AutoStabilizerService"),

  mt64(0),
  random(0.0, 0.05)
{
  this->m_service0_.setComp(this);
  
  exec_tm = 0.0;
  epsilon = 1e-6;

  // state管理の都合上、dtの整数倍にしておく
  ff_motion_time = 0.30;
  fb_motion_time = 0.45; // 本当はfbの結果から
  motion_time = ff_motion_time;

  motion_state = 0; // 0:停止時, 1:ff, 2:fb

  ball_r = 0.1225; // 他のボールも使えるようにするなら上から与えられるようにする??

  /* -- targetContactBallState -- */
  // todo: 上位から変更できるようにする
  targetContactBallState.resize(2);
  // 右手
  // pos
  targetContactBallState[0].pos.x = 0.60;
  targetContactBallState[0].pos.y = -0.45;
  targetContactBallState[0].pos.z = 1.1775;
  targetContactBallState[0].vel.x = 0.0;
  targetContactBallState[0].vel.y = 0.0;
  targetContactBallState[0].vel.z = 0.0;

  // 左手
  targetContactBallState[1].pos.x = 0.60;
  targetContactBallState[1].pos.y = 0.45;
  targetContactBallState[1].pos.z = 1.1775;
  targetContactBallState[1].vel.x = 0.0;
  targetContactBallState[1].vel.y = 0.0;
  targetContactBallState[1].vel.z = 0.0;

  /* -- nextContactBallState -- */
  nextContactBallState = targetContactBallState[0];


  

  /* -- targetContactEEPos/Rpy -- */
  // posはボールの半径+ハンドの厚み分zを高くする
  // rpyは一旦手動で設定
  targetContactEEPos.resize(3, std::vector<double>(3)); // x,y,z
  targetContactEERpy.resize(3, std::vector<double>(3)); // r,p,y
  // 右手
  targetContactEEPose[0].data.position.x = targetContactBallState[0].pos.x;
  targetContactEEPose[0].data.position.y = targetContactBallState[0].pos.y;
  targetContactEEPose[0].data.position.z = targetContactBallState[0].pos.z + ball_r;
  targetContactEEPose[0].data.orientation.r = -M_PI/2.0;
  targetContactEEPose[0].data.orientation.p = -M_PI/8.0;
  targetContactEEPose[0].data.orientation.y = 0.0;
  // // 左手
  // targetContactEEPose[1].data.position.x = targetContactBallState[1].pos.x;
  // targetContactEEPose[1].data.position.y = targetContactBallState[1].pos.y;
  // targetContactEEPose[1].data.position.z = targetContactBallState[1].pos.x + ball_r;
  // targetContactEEPose[1].data.orientation.r = -M_PI/2.0; // M_PI/2.0かも
  // targetContactEEPose[1].data.orientation.p = -M_PI/8.0;
  // targetContactEEPose[1].data.orientation.y = 0.0;

  // ffのmotionのパラメタ初期化
  // start, rangeの順番
  rarm_pos_range = {{targetContactEEPose[0].data.position.x, 0.020},
		    {targetContactEEPose[0].data.position.y, 0.0},
		    {targetContactEEPose[0].data.position.z, -0.30}};
  rarm_rpy_range = {{targetContactEEPose[0].data.orientation.r, 0.0},
		    {targetContactEEPose[0].data.orientation.p, M_PI/4.0},
		    {targetContactEEPose[0].data.orientation.y, 0.0}};
  // 左手は動かさない想定で一旦実装する
  larm_pos_range = {{0.0, 0.0}, {0.45, 0.0}, {0.68, 0.0}};
  larm_rpy_range = {{0.0, 0.0}, {M_PI/2.5, 0.0}, {0.0, 0.0}};

  // resize
  bound_point.resize(3);
  dummyBallPos.resize(3);
    
  // flag
  motionEnd_flag = false;
  last_motion = false;
  stateChange_flag = false;
  loop_init = true;

  startDribbleMode_flag = false;
  startDribbleMotion_flag = false;
}

RTC::ReturnCode_t BasketballMotionController::onInitialize(){
  // InPort
  addInPort("nowObjStateIn", this->m_nowObjStateIn_);
  addInPort("predObjStateIn", this->m_predObjStateIn_);
  addInPort("fbFlagIn", this->m_fbFlagIn_);

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

  if (!startDribbleMotion_flag) {    
    // startDribbleMotionが呼ばれてflag=trueになるまではinitPoseを送り続ける
    initPose();
  } else {
    // readInPortData
    readInPortData();
    
    // 状態の管理,更新
    stateManager();
    
    // update param
    if (stateChange_flag) {
      updateParam();
      stateChange_flag = false; // とりあえずテストで切り替わったタイミングに一回paramを更新するのを試す
    }

    // 軌道の生成
    genTargetEEPose();
    
  }
  
  // writeOutPortData
  writeOutPortData();
  
  return RTC::RTC_OK;
}

void BasketballMotionController::readInPortData() {
  // nowObjState
  if (m_nowObjStateIn_.isNew()) {
    m_nowObjStateIn_.read();

    // デバッグ用
    std::cout << "nowObjState.pos.z: " << m_nowObjState_.pos.z << std::endl;
  }
  // predObjState
  if (m_predObjStateIn_.isNew()) {
    m_predObjStateIn_.read();
  }
  
  // // fbFlag
  // if (m_fbFlagIn_.isNew()) {
  //   m_fbFlagIn_.read();
  // }
}

void BasketballMotionController::writeOutPortData() {
  // eePose
  for(int i=0;i<4;i++){
    // // position
    // m_eePose_[i].data.position.x = targetEEPose[i].data.position.x;
    // m_eePose_[i].data.position.y = targetEEPose[i].data.position.y;
    // m_eePose_[i].data.position.z = targetEEPose[i].data.position.z;
    // // orientation
    // m_eePose_[i].data.orientation.r = targetEEPose[i].data.orientation.r;
    // m_eePose_[i].data.orientation.p = targetEEPose[i].data.orientation.p;
    // m_eePose_[i].data.orientation.y = targetEEPose[i].data.orientation.y;

    // position
    m_eePose_[i].data.position.x = targetEEPose[i][0];
    m_eePose_[i].data.position.y = targetEEPose[i][1];
    m_eePose_[i].data.position.z = targetEEPose[i][2];
    // orientation
    m_eePose_[i].data.orientation.r = targetEEPose[i][3];
    m_eePose_[i].data.orientation.p = targetEEPose[i][4];
    m_eePose_[i].data.orientation.y = targetEEPose[i][5];
    
    m_eePoseOut_[i]->write();
  }
}

void BasketballMotionController::dummyObjPos() {
  dummyBallPos[0] = rarm_pos_range[0][0];
  dummyBallPos[1] = rarm_pos_range[1][0];
  dummyBallPos[2] = 1.3 + random(mt64);
  std::cout << "dummy_z:" << dummyBallPos[2] << std::endl;
}

void BasketballMotionController::stateManager() {
  // todo: ここを変更
  // サイクル終了判定
  if (std::fabs(exec_tm - (motion_time + dt)) < epsilon) { // 
    exec_tm = 0.0;
    stateChange_flag = true;
    loop_init = true;

    // ドリブル終了判定
    if (motionEnd_flag && last_motion && (motion_state==2)) {
      startDribbleMotion_flag = false;
      motionEnd_flag = false;
      last_motion = false;
    }
  }

  // 時間の更新
  if (!loop_init) exec_tm += dt;
  loop_init = false;
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
      // dummyObjPos();
      // rarm_pos_range[2][0] = dummyBallPos[2];
    } else if (motion_state==2) {
      // ff_mode
      motion_state = 1;
      motion_time = ff_motion_time;
      
      // rangeの設定
      // startは最高到達点でcontactしたところ
      // goalはどうする??とりあえず変更なし??
      // rarm_pos_range[0][0] = targetContactEEPose[0].data.position.x;
      // rarm_pos_range[1][0] = targetContactEEPose[1].data.position.y;
      // rarm_pos_range[2][0] = targetContactEEPose[2].data.position.z;
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
}

// 三角関数によるフィードフォワード軌道生成
// todo: ここをもう少しそれっぽいものにしたい
void BasketballMotionController::ffTargetEEPose() {
  // x,y = start + 0.5 * range * (1 - cos(2pi/T)*t)
  //   z = start + 0.5 * range * (1 - cos(pi/T)*t)

  // rarm
  // position
  targetEEPose[2][0] = rarm_pos_range[0][0] + 0.5*rarm_pos_range[0][1]*(1 - std::cos((2*M_PI/motion_time)*exec_tm));
  targetEEPose[2][1] = rarm_pos_range[1][0] + 0.5*rarm_pos_range[1][1]*(1 - std::cos((2*M_PI/motion_time)*exec_tm));
  targetEEPose[2][2] = rarm_pos_range[2][0] + 0.5*rarm_pos_range[2][1]*(1 - std::cos((M_PI/motion_time)*exec_tm));
  // orientation
  targetEEPose[2][3] = rarm_rpy_range[0][0] + 0.5*rarm_rpy_range[0][1]*(1 - std::cos((M_PI/motion_time)*exec_tm));
  targetEEPose[2][4] = rarm_rpy_range[1][0] + 0.5*rarm_rpy_range[1][1]*(1 - std::cos((M_PI/motion_time)*exec_tm));
  targetEEPose[2][5] = rarm_rpy_range[2][0] + 0.5*rarm_rpy_range[2][1]*(1 - std::cos((M_PI/motion_time)*exec_tm));
  
  // larm
  // 今は同じ値を送り続ける
  targetEEPose[3][0] = larm_pos_range[0][0];
  targetEEPose[3][1] = larm_pos_range[1][0];
  targetEEPose[3][2] = larm_pos_range[2][0];
  targetEEPose[3][3] = larm_rpy_range[0][0];
  targetEEPose[3][4] = larm_rpy_range[1][0];
  targetEEPose[3][5] = larm_rpy_range[2][0];
  
  // // rarm
  // // position
  // targetEEPose[2].data.position.x = rarm_pos_range[0][0] + 0.5*rarm_pos_range[0][1]*(1 - std::cos((2*M_PI/motion_time)*exec_tm));
  // targetEEPose[2].data.position.y = rarm_pos_range[1][0] + 0.5*rarm_pos_range[1][1]*(1 - std::cos((2*M_PI/motion_time)*exec_tm));
  // targetEEPose[2].data.position.z = rarm_pos_range[2][0] + 0.5*rarm_pos_range[2][1]*(1 - std::cos((M_PI/motion_time)*exec_tm));
  // // orientation
  // targetEEPose[2].data.orientation.r = rarm_rpy_range[0][0] + 0.5*rarm_rpy_range[0][1]*(1 - std::cos((M_PI/motion_time)*exec_tm));
  // targetEEPose[2].data.orientation.p = rarm_rpy_range[1][0] + 0.5*rarm_rpy_range[1][1]*(1 - std::cos((M_PI/motion_time)*exec_tm));
  // targetEEPose[2].data.orientation.y = rarm_rpy_range[2][0] + 0.5*rarm_rpy_range[2][1]*(1 - std::cos((M_PI/motion_time)*exec_tm));
  
  // // larm
  // // 今は同じ値を送り続ける
  // targetEEPose[3].data.position.x = larm_pos_range[0][0];
  // targetEEPose[3].data.position.y = larm_pos_range[1][0];
  // targetEEPose[3].data.position.z = larm_pos_range[2][0];
  // targetEEPose[3].data.orientation.r = larm_rpy_range[0][0];
  // targetEEPose[3].data.orientation.p = larm_rpy_range[1][0];
  // targetEEPose[3].data.orientation.y = larm_rpy_range[2][0];
  
  // legは全て0を送るので何もしない

  /* ---------------------------------------------------------- */

  // 値の保持
  prev_EEPose = targetEEPose;
  
}

void calcEEVelocity() {
  // pos
  EEVel[2][0] = (targetEEPose[2].data.position.x - prevEEPose[2].data.position.x) / dt;
  EEVel[2][1] = (targetEEPose[2].data.position.y - prevEEPose[2].data.position.y) / dt;
  EEVel[2][2] = (targetEEPose[2].data.position.z - prevEEPose[2].data.position.z) / dt;
  // rpy
  EEVel[2][3] = (targetEEPose[2].data.orientation.r - prevEEPose[2].data.orientation.r) / dt;
  EEVel[2][4] = (targetEEPose[2].data.orientation.p - prevEEPose[2].data.orientation.p) / dt;
  EEVel[2][5] = (targetEEPose[2].data.orientation.y - prevEEPose[2].data.orientation.y) / dt;
  
  // for (int i=0; i<4; i++) {
  //   // pos
  //   EEVel[i][0] = (targetEEPose[i].data.position.x - prevEEPose[i].data.position.x) / dt;
  //   EEVel[i][1] = (targetEEPose[i].data.position.y - prevEEPose[i].data.position.y) / dt;
  //   EEVel[i][2] = (targetEEPose[i].data.position.z - prevEEPose[i].data.position.z) / dt;
  //   // rpy
  //   EEVel[i][3] = (targetEEPose[i].data.orientation.r - prevEEPose[i].data.orientation.r) / dt;
  //   EEVel[i][4] = (targetEEPose[i].data.orientation.p - prevEEPose[i].data.orientation.p) / dt;
  //   EEVel[i][5] = (targetEEPose[i].data.orientation.y - prevEEPose[i].data.orientation.y) / dt;
  // }
}

void calcEEAcceleratin() {
  EEAcc = (EEVel - prevEEAcc) / dt;
}

// 多項式補間をするための初期条件と終端条件の設定 
void setPolynomialConditions() {

  tf = ;

  
  /* -- pos start -- */
  // x
  pos_start_conditions[0].val = prevEEPose[2].data.position.x;
  pos_start_conditions[0].first_der = EEVel[2][0];
  pos_start_conditions[0].second_der = ;
  // y
  pos_start_conditions[1].val = prevEEPose[2].data.position.y;
  pos_start_conditions[1].first_der = EEVel[2][1];
  pos_start_conditions[1].second_der = ;
  // z
  pos_start_conditions[2].val = prevEEPose[2].data.position.z;
  pos_start_conditions[2].first_der = EEVel[2][2];
  pos_start_conditions[2].second_der = ;

  /* -- rpy start -- */
  // r
  rpy_start_conditions[0].val = prevEEPose[2].data.orientation.r;
  rpy_start_conditions[0].first_der = EEVel[2][3];
  rpy_start_conditions[0].second_der = ;
  // p
  rpy_start_conditions[1].val = prevEEPose[2].data.orientation.p;
  rpy_start_conditions[1].first_der = EEVel[2][4];
  rpy_start_conditions[1].second_der = ;
  // y
  rpy_start_conditions[2].val = prevEEPose[2].data.orientation.y;
  rpy_start_conditions[2].first_der = EEVel[2][5];
  rpy_start_conditions[2].second_der = ;

  /* -- pos end -- */
  // x
  pos_end_conditions[0].val = ;
  pos_end_conditions[0].first_der = ;
  pos_end_conditions[0].second_der = ;
  // y
  pos_end_conditions[1].val = ;
  pos_end_conditions[1].first_der = ;
  pos_end_conditions[1].second_der = ;
  // z
  pos_end_conditions[2].val = ;
  pos_end_conditions[2].first_der = ;
  pos_end_conditions[2].second_der = ;

  /* -- rpy end -- */
  // r
  rpy_end_conditions[0].val = ;
  rpy_end_conditions[0].first_der = ;
  rpy_end_conditions[0].second_der = ;
  // p
  rpy_end_conditions[1].val = ;
  rpy_end_conditions[1].first_der = ;
  rpy_end_conditions[1].second_der = ;
  //y
  rpy_end_conditions[2].val = ;
  rpy_end_conditions[2].first_der = ;
  rpy_end_conditions[2].second_der = ;
  
}

// 多項式補間によるフィードバック軌道生成
// 初期位置/速度, 終端位置/速度の4条件から3次関数の補間軌道を作る
void BasketballMotionController::fbTargetEEPose() {
  // 初期条件と終端条件の設定
  setPolynomialConditions();

  // 5次多項式の係数の計算
    
  
  ///////////

  // 昔のver
  // x,y = start + 0.5 * range * (1 - cos(2pi/T)*t)
  //   z = start + 0.5 * range * (1 - cos(pi/T)*t)

  // rarm
  // position
  targetEEPose[2].data.position.x = rarm_pos_range[0][0] + 0.5*rarm_pos_range[0][1]*(1 - std::cos((2*M_PI/motion_time)*exec_tm));
  targetEEPose[2].data.position.y = rarm_pos_range[1][0] + 0.5*rarm_pos_range[1][1]*(1 - std::cos((2*M_PI/motion_time)*exec_tm));
  targetEEPose[2].data.position.z = rarm_pos_range[2][0] + 0.5*rarm_pos_range[2][1]*(1 - std::cos((M_PI/motion_time)*exec_tm + M_PI));
  // orientation
  targetEEPose[2].data.orientation.r = rarm_rpy_range[0][0] + 0.5*rarm_rpy_range[0][1]*(1 - std::cos((M_PI/motion_time)*exec_tm + M_PI));
  targetEEPose[2].data.orientation.p = rarm_rpy_range[1][0] + 0.5*rarm_rpy_range[1][1]*(1 - std::cos((M_PI/motion_time)*exec_tm + M_PI));
  targetEEPose[2].data.orientation.y = rarm_rpy_range[2][0] + 0.5*rarm_rpy_range[2][1]*(1 - std::cos((M_PI/motion_time)*exec_tm + M_PI));
  
  // larm
  // 今は同じ値を送り続ける
  targetEEPose[3][0] = larm_pos_range[0][0];
  targetEEPose[3][1] = larm_pos_range[1][0];
  targetEEPose[3][2] = larm_pos_range[2][0];
  targetEEPose[3][3] = larm_rpy_range[0][0];
  targetEEPose[3][4] = larm_rpy_range[1][0];
  targetEEPose[3][5] = larm_rpy_range[2][0];
  
  // legは全て0を送るので何もしない

}
  
void BasketballMotionController::initPose(){
  // 単位は[m]
  // 右手
  targetEEPose[2][0] = rarm_pos_range[0][0];
  targetEEPose[2][1] = rarm_pos_range[1][0];
  targetEEPose[2][2] = rarm_pos_range[2][0];
  targetEEPose[2][3] = rarm_rpy_range[0][0];
  targetEEPose[2][4] = rarm_rpy_range[1][0];
  targetEEPose[2][5] = rarm_rpy_range[2][0];
  // 左手
  targetEEPose[3][0] = larm_pos_range[0][0];
  targetEEPose[3][1] = larm_pos_range[1][0];
  targetEEPose[3][2] = larm_pos_range[2][0];
  targetEEPose[3][3] = larm_rpy_range[0][0];
  targetEEPose[3][4] = larm_rpy_range[1][0];
  targetEEPose[3][5] = larm_rpy_range[2][0];
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

  usleep(2000000); // 1秒以上待たないとWholeBodyMasterSlaveが呼ばれない
  m_autoStabilizerService0_->startWholeBodyMasterSlave();
  
  return true;
}

bool BasketballMotionController::stopDribbleMode(){
  resetParam();
  exec_tm = 0.0;

  usleep(2000000); // 1秒以上待たないとWholeBodyMasterSlaveが呼ばれない
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

bool BasketballMotionController::setBasketballMotionControllerParam(const double data){
  
}

bool BasketballMotionController::getBasketballMotionControllerParam(const double data){
  
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
