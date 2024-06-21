#include "BasketballMotionController.h"

#include <random>
#include <cmath>

BasketballMotionController::BasketballMotionController(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_nowObjStateIn_("nowObjStateIn", m_nowObjState_),
  m_predObjStateIn_("predObjStateIn", m_predObjState_),
  m_fbFlagIn_("fbFlagIn", m_fbFlag_),
  m_qRefIn_("qRefIn", m_qRef_),
  
  m_BasketballMotionControllerServicePort_("BasketballMotionControllerService"),
  m_AutoStabilizerServicePort_("AutoStabilizerService"),

  mt64(0),
  random1(-0.02, 0.02),
  random2(0.06, 0.10)  
{
  this->m_service0_.setComp(this);
}

RTC::ReturnCode_t BasketballMotionController::onInitialize() {
  // InPort
  addInPort("nowObjStateIn", this->m_nowObjStateIn_);
  addInPort("predObjStateIn", this->m_predObjStateIn_);
  addInPort("fbFlagIn", this->m_fbFlagIn_);
  addInPort("qRefIn", this->m_qRefIn_);

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
  // log用
  // rarm point
  m_eePointOut_.resize(eeNames.size());
  m_eePoint_.resize(eeNames.size());
  for(size_t i=0;i<eeNames.size();i++){
    std::string portName = eeNames[i] + "PointOut";
    m_eePointOut_[i] = std::make_unique<RTC::OutPort<RTC::TimedPoint3D> >(portName.c_str(), m_eePoint_[i]);
    addOutPort(portName.c_str(), *(m_eePointOut_[i]));
  }
  // rarm orientation
  m_eeOrientationOut_.resize(eeNames.size());
  m_eeOrientation_.resize(eeNames.size());
  for(size_t i=0;i<eeNames.size();i++){
    std::string portName = eeNames[i] + "OrientationOut";
    m_eeOrientationOut_[i] = std::make_unique<RTC::OutPort<RTC::TimedOrientation3D> >(portName.c_str(), m_eeOrientation_[i]);
    addOutPort(portName.c_str(), *(m_eeOrientationOut_[i]));
  }

  exec_tm = 0.0;
  epsilon = 1e-6;

  ff_motion_time = 0.35; // state管理の都合上、dtの整数倍にしておく
  // ff_motion_time = 1.0; // state管理の都合上、dtの整数倍にしておく
  motion_time = ff_motion_time; 

  motion_state = 0; // 0:停止時, 1:ff, 2:fb

  hand_offset.resize(6);
  hand_offset[0] = -0.10;
  hand_offset[1] = 0.0;
  hand_offset[2] = 0.1225;
  hand_offset[3] = 0.0;
  hand_offset[4] = 0.0;
  hand_offset[5] = 0.0;

  maxDribbleReach = 0.20;

  /* -- targetContactBallState -- */
  // todo: 上位から変更できるようにする
  targetContactBallState.resize(2);
  // 右手
  // pos
  targetContactBallState[0].pos.x = 0.70;
  targetContactBallState[0].pos.y = -0.45;
  targetContactBallState[0].pos.z = 1.1775;
  targetContactBallState[0].vel.x = 0.0;
  targetContactBallState[0].vel.y = 0.0;
  targetContactBallState[0].vel.z = 0.0;
  // // 左手
  // targetContactBallState[1].pos.x = 0.60;
  // targetContactBallState[1].pos.y = 0.45;
  // targetContactBallState[1].pos.z = 1.1775;
  // targetContactBallState[1].vel.x = 0.0;
  // targetContactBallState[1].vel.y = 0.0;
  // targetContactBallState[1].vel.z = 0.0;

  /* -- nextContactBallState -- */
  // fbの値で更新する
  nextContactBallState = targetContactBallState[0];

  /* -- targetContactEEPos/Rpy -- */
  // posはボールの半径+ハンドの厚み分zを高くする, 指先がボールの中心に来るようにxをずらす
  // rpyは一旦手動で設定
  // nextContactBallStateから決める
  targetContactEEPose = Eigen::MatrixXd(4,6);
  targetContactEEVel  = Eigen::MatrixXd(4,6);
  targetContactEEAcc  = Eigen::MatrixXd(4,6);
  // 右手
  targetContactEEPose(2,0) = targetContactBallState[0].pos.x + hand_offset[0];
  targetContactEEPose(2,1) = targetContactBallState[0].pos.y + hand_offset[1];
  targetContactEEPose(2,2) = targetContactBallState[0].pos.z + hand_offset[2];
  targetContactEEPose(2,3) = -M_PI/2.0 + hand_offset[3];
  targetContactEEPose(2,4) = -M_PI/8.0 + hand_offset[4];
  targetContactEEPose(2,5) = 0.0 + hand_offset[5];
  // // 左手
  // targetContactEEPose(3,0) = targetContactBallState[1].pos.x;
  // targetContactEEPose(3,1) = targetContactBallState[1].pos.y;
  // targetContactEEPose(3,2) = targetContactBallState[1].pos.x + ball_r;
  // targetContactEEPose(3,3) = -M_PI/2.0; // M_PI/2.0かも
  // targetContactEEPose(3,4) = -M_PI/8.0;
  // targetContactEEPose(3,5) = 0.0;

  // ffのmotionのパラメタ初期化
  targetEEPose = Eigen::MatrixXd(4,6);
  prevEEPose   = Eigen::MatrixXd(4,6);
  EEVel        = Eigen::MatrixXd(4,6);
  prevEEVel    = Eigen::MatrixXd(4,6);
  EEAcc        = Eigen::MatrixXd(4,6);
  
  // start, rangeの順番
  rarm_pos_params = {{targetContactEEPose(2,0), 0.020},
		     {targetContactEEPose(2,1), 0.0},
		     {targetContactEEPose(2,2), -0.30}};
  rarm_rpy_params = {{targetContactEEPose(2,3), 0.0},
		     {targetContactEEPose(2,4), M_PI/4.0},
		     {targetContactEEPose(2,5), 0.0}};
  // 左手は動かさない想定で一旦実装する
  larm_pos_params = {{0.0, 0.0}, {0.45, 0.0}, {0.68, 0.0}};
  larm_rpy_params = {{0.0, 0.0}, {M_PI/2.5, 0.0}, {0.0, 0.0}};
  
  // resize
  bound_point.resize(3);
  dummyBallPos.resize(3);
    
  // flag
  motionEnd_flag = false;
  last_motion = false;
  stateChange_flag = false;
  loop_init = true;
  fb_initialize = true;
  fb_flag = false;

  test_flag = true;

  startDribbleMode_flag = false;
  startDribbleMotion_flag = false;

  // prevEEPoseをinitPoseに合わせる
  // rarm
  prevEEPose(2,0) = rarm_pos_params[0][0];
  prevEEPose(2,1) = rarm_pos_params[1][0];
  prevEEPose(2,2) = rarm_pos_params[2][0];
  prevEEPose(2,3) = rarm_rpy_params[0][0];
  prevEEPose(2,4) = rarm_rpy_params[1][0];
  prevEEPose(2,5) = rarm_rpy_params[2][0];
  // larm
  prevEEPose(3,0) = larm_pos_params[0][0];
  prevEEPose(3,1) = larm_pos_params[1][0];
  prevEEPose(3,2) = larm_pos_params[2][0];
  prevEEPose(3,3) = larm_rpy_params[0][0];
  prevEEPose(3,4) = larm_rpy_params[1][0];
  prevEEPose(3,5) = larm_rpy_params[2][0];

  // fb
  polynomial_degree = 5;
  rarm_fb_theta.resize(6);
  larm_fb_theta.resize(6);
  start_conditions.resize(6);
  end_conditions.resize(6);
    
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
    
    readInPortData();
    
    // 状態の管理,更新
    stateManager();

    // パラメタの更新
    if (stateChange_flag) {
      updateParams();
      stateChange_flag = false; // とりあえずテストで切り替わったタイミングに一回paramを更新するのを試す
    }

    // 軌道の生成
    genTargetEEPose();
    
  }
  
  writeOutPortData();
  
  return RTC::RTC_OK;
}

void BasketballMotionController::readInPortData() {
  // nowObjState
  if (m_nowObjStateIn_.isNew()) {
    m_nowObjStateIn_.read();
  }
  // predObjState
  if (m_predObjStateIn_.isNew()) {
    m_predObjStateIn_.read();

    // 0.002の倍数に丸める
    contact_tm = round(1e9*m_predObjState_.tm.nsec/dt)*dt;
  }
  
  // // fbFlag
  // if (m_fbFlagIn_.isNew()) {
  //   m_fbFlagIn_.read();
  //   fb_flag = true;
  // }

  // log用
  if (m_qRefIn_.isNew()) {
    m_qRefIn_.read();
  }
}

void BasketballMotionController::writeOutPortData() {
  // eePose
  for(int i=0;i<4;i++){
    // position
    m_eePose_[i].data.position.x = targetEEPose(i,0);
    m_eePose_[i].data.position.y = targetEEPose(i,1);
    m_eePose_[i].data.position.z = targetEEPose(i,2);
    // orientation
    m_eePose_[i].data.orientation.r = targetEEPose(i,3);
    m_eePose_[i].data.orientation.p = targetEEPose(i,4);
    m_eePose_[i].data.orientation.y = targetEEPose(i,5);

    m_eePose_[i].tm = m_qRef_.tm;
    
    m_eePoseOut_[i]->write();
  }

  // log用
  for(int i=0;i<4;i++){
    m_eePoint_[i].data = m_eePose_[i].data.position;
    m_eePoint_[i].tm = m_qRef_.tm;
    
    m_eeOrientation_[i].data = m_eePose_[i].data.orientation;
    m_eeOrientation_[i].tm = m_qRef_.tm;
    
    m_eePointOut_[i]->write();
    m_eeOrientationOut_[i]->write();
  }
}

void BasketballMotionController::dummyObjPos() {
  dummyBallPos[0] = rarm_pos_params[0][0];
  dummyBallPos[1] = rarm_pos_params[1][0];
  dummyBallPos[2] = 1.3 + random1(mt64);
  std::cout << "dummy_z:" << dummyBallPos[2] << std::endl;
}

void BasketballMotionController::stateManager() {

  // fb test
  if (motion_state == 1 && (fabs(exec_tm - (ff_motion_time + 0.10 + dt)) < epsilon)) fb_flag = true;
  
  // stateが変わるタイミング
  // 1: motion_state=1のときにfbのフラグが立ったとき
  // 2: motion_state=2のときにmotion_time分時間が経過したとき
  if ((motion_state == 1 && fb_flag) || (motion_state == 2 && (fabs(exec_tm - motion_time + dt) < epsilon))) {
        
    exec_tm = 0.0;
    stateChange_flag = true;
    loop_init = true;

    fb_flag = false;

    // ドリブル終了判定
    if (motionEnd_flag && last_motion && (motion_state==2)) {
      startDribbleMotion_flag = false;
      motionEnd_flag = false;
      last_motion = false;
    }
  }

  // // 旧ver
  // if (std::fabs(exec_tm - (motion_time + dt)) < epsilon) {
  //   exec_tm = 0.0;
  //   stateChange_flag = true;
  //   loop_init = true;

  //   // ドリブル終了判定
  //   if (motionEnd_flag && last_motion && (motion_state==2)) {
  //     startDribbleMotion_flag = false;
  //     motionEnd_flag = false;
  //     last_motion = false;
  //   }
  // }

  // 時間の更新
  if (!loop_init) exec_tm += dt;
  loop_init = false;
}

void BasketballMotionController::updateParams() {
  if (motionEnd_flag) {
    // 動作停止モードへ以降
    if (motion_state==1) {
      motion_state = 2;
      motion_time = 1.0;
      fb_initialize = true;
      
    } else if (motion_state==2) {
      motion_state = 1;
      motion_time = 1.0;

      setFeedForwardParams();
      
    }

    last_motion = true;
    
  } else {
    if (motion_state==1) {
      motion_state = 2;
      // motion_time = contact_tm;

      // test
      // 実際のcontact_tmにもこの処理が必要かも
      double random_value = 0.50 + random1(mt64);
      // double random_value = 1.50 + random1(mt64);
      double rounded_value = round(random_value/dt) * dt;
      motion_time = rounded_value;
  
      fb_initialize = true;
      
    } else if (motion_state==2) {
      motion_state = 1;
      motion_time = ff_motion_time; // ここも計算で更新するべき

      setFeedForwardParams();
    }
  }  
}

/* -- フィードフォワードのモーションを計算するためのパラメタの設定 -- */
// start: targetContactEEPose
// range: targetContactBallStateとの関係性から決めるorマップを作るようにしたいが、一旦実装はしない
void BasketballMotionController::setFeedForwardParams() {
  /* -- start -- */
  // pos
  rarm_pos_params[0][0] = targetContactEEPose(2,0);
  rarm_pos_params[1][0] = targetContactEEPose(2,1);
  rarm_pos_params[2][0] = targetContactEEPose(2,2);
  // orientation
  rarm_rpy_params[0][0] = targetContactEEPose(2,3);
  rarm_rpy_params[1][0] = targetContactEEPose(2,4);
  rarm_rpy_params[2][0] = targetContactEEPose(2,5);

  /* -- range -- */
}

/* -- エンドエフェクタのpose,vel,accの計算 -- */
void BasketballMotionController::genTargetEEPose() {
  // poseの計算
  if (motion_state==1) {
    ffTargetEEPose();
  } else if (motion_state==2) {
    fbTargetEEPose();
  }

  // vel、accの計算
  EEVel = (targetEEPose - prevEEPose) / dt;
  EEAcc = (EEVel - prevEEVel) / dt;
  
  // 値の保持
  prevEEPose = targetEEPose;
  prevEEVel = EEVel;
}

/* -- フィードフォワード軌道生成 -- */
// 現状は三角関数による補間
void BasketballMotionController::ffTargetEEPose() {
  // x,y = start + 0.5 * range * (1 - cos(2pi/T)*t)
  //   z = start + 0.5 * range * (1 - cos(pi/T)*t)

  // legは全て0を送るので何もしない
  
  // rarm
  // position
  targetEEPose(2,0) = rarm_pos_params[0][0] + 0.5*rarm_pos_params[0][1]*(1 - cos((2*M_PI/motion_time)*exec_tm));
  targetEEPose(2,1) = rarm_pos_params[1][0] + 0.5*rarm_pos_params[1][1]*(1 - cos((2*M_PI/motion_time)*exec_tm));
  targetEEPose(2,2) = rarm_pos_params[2][0] + 0.5*rarm_pos_params[2][1]*(1 - cos((M_PI/motion_time)*exec_tm));
  // orientation
  targetEEPose(2,3) = rarm_rpy_params[0][0] + 0.5*rarm_rpy_params[0][1]*(1 - cos((M_PI/motion_time)*exec_tm));
  targetEEPose(2,4) = rarm_rpy_params[1][0] + 0.5*rarm_rpy_params[1][1]*(1 - cos((M_PI/motion_time)*exec_tm));
  targetEEPose(2,5) = rarm_rpy_params[2][0] + 0.5*rarm_rpy_params[2][1]*(1 - cos((M_PI/motion_time)*exec_tm));
  
  // larm
  // 今は同じ値を送り続ける
  targetEEPose(3,0) = larm_pos_params[0][0];
  targetEEPose(3,1) = larm_pos_params[1][0];
  targetEEPose(3,2) = larm_pos_params[2][0];
  targetEEPose(3,3) = larm_rpy_params[0][0];
  targetEEPose(3,4) = larm_rpy_params[1][0];
  targetEEPose(3,5) = larm_rpy_params[2][0];
  
}

/* -- 次の目標接触ハンド状態の計算 -- */
// predObjState -> nextContactBallState -> targetContactEEState(Pose/Vel/Acc)
void BasketballMotionController::calcContactEEPose() {
  // 将来的にここに代入以外の処理を入れるかもしれない
  // nextContactBallState = m_predObjState_;

  // test
  nextContactBallState.pos.x = 0.70;
  if (test_flag) {
    nextContactBallState.pos.y = -0.45 + 0.05;
    test_flag = false;
  } else {
    nextContactBallState.pos.y = -0.45 - 0.05;
    test_flag = true;
  }
  nextContactBallState.pos.z = 1.1775;

  // maxDribbleReach内に収まっているかどうかのチェック
  double dist
    = sqrt(pow((targetContactBallState[0].pos.x - nextContactBallState.pos.x), 2)
           + pow((targetContactBallState[0].pos.y - nextContactBallState.pos.y), 2)
	   + pow((targetContactBallState[0].pos.z - nextContactBallState.pos.z), 2));

  std::cout << "dist: " << dist << std::endl;

  if (dist > maxDribbleReach) {
    // targetContactEEPose等を更新せず、motionEnd_flagををtrueにする
    motionEnd_flag = true;
    std::cout << "Cannot dribble because the predicted ball position is outside the dribbling range" << std::endl;
    
  } else {
    /* -- Pose -- */
    // pos
    targetContactEEPose(2,0) = nextContactBallState.pos.x + hand_offset[0];
    targetContactEEPose(2,1) = nextContactBallState.pos.y + hand_offset[1];
    targetContactEEPose(2,2) = nextContactBallState.pos.z + hand_offset[2];
    // orientation
    // r,pは一旦固定
    targetContactEEPose(2,3) = -M_PI/2.0 + hand_offset[3];
    targetContactEEPose(2,4) = -M_PI/8.0 + hand_offset[4];
    // yは最初のtargetContactEEPoseの位置を基準にずれた分補正する
    double dy = (targetContactBallState[0].pos.y + 0.0) - prevEEPose(2,1);
    targetContactEEPose(2,5) = atan(dy/targetContactEEPose(2,0)) + hand_offset[5];
    
    /* -- Vel -- */
    // pos
    targetContactEEVel(2,0) = 0.0;
    targetContactEEVel(2,1) = 0.0;
    targetContactEEVel(2,2) = 0.0;
    // orientation
    targetContactEEVel(2,3) = 0.0;
    targetContactEEVel(2,4) = 0.0;
    targetContactEEVel(2,5) = 0.0;
    
    /* -- Acc -- */
    // pos
    targetContactEEAcc(2,0) = 0.5*rarm_pos_params[0][1]*pow(2*M_PI/motion_time, 2);
    targetContactEEAcc(2,1) = 0.5*rarm_pos_params[1][1]*pow(2*M_PI/motion_time, 2);
    targetContactEEAcc(2,2) = 0.5*rarm_pos_params[2][1]*pow(M_PI/motion_time, 2);
    // orientation
    targetContactEEAcc(2,3) = 0.5*rarm_rpy_params[0][1]*pow(M_PI/motion_time, 2);
    targetContactEEAcc(2,4) = 0.5*rarm_rpy_params[1][1]*pow(M_PI/motion_time, 2);
    targetContactEEAcc(2,5) = 0.5*rarm_rpy_params[2][1]*pow(M_PI/motion_time, 2); 
  }
}


/* -- 多項式補間をするための初期条件と終端条件の設定 -- */
// 右手だけ
void BasketballMotionController::setPolynomialConditions() {
  
  calcContactEEPose();
  
  for (int i=0; i<6; i++) {
    // start
    // 今の状態から決まる
    start_conditions[i].val        = prevEEPose(2,i);
    start_conditions[i].first_der  = EEVel(2,i);
    start_conditions[i].second_der = EEAcc(2,i);
    
    // end
    // fbの値から決まる
    end_conditions[i].val        = targetContactEEPose(2,i);
    end_conditions[i].first_der  = targetContactEEVel(2,i);
    end_conditions[i].second_der = targetContactEEAcc(2,i);
  }
  
}

/* -- フィードバック軌道生成 --*/
// 時間の多項式による補間
// 加速度連続まで考慮して5次
void BasketballMotionController::fbTargetEEPose() {
  // とりあえずフィードバックが入った最初の値で軌道を生成するようにする
  if (fb_initialize) {    
    // 初期条件と終端条件の設定
    setPolynomialConditions();

    // 5次多項式の係数の計算
    // とりあえず右手だけ
    for (int i=0; i<6; i++) {
      // rarm_fb_theta[i] = fbInterpolator.calcCoefficients(start_conditions[i], end_conditions[i], contact_tm);
      rarm_fb_theta[i] = fbInterpolator.calcCoefficients(start_conditions[i], end_conditions[i], motion_time);
    }

    fb_initialize = false;
  }

  // legは全て0を送るので何もしない

  // rarm
  targetEEPose.setZero();
  for (int i=0; i<6; i++) {
    for (int j=0; j<polynomial_degree+1; j++) {
    targetEEPose(2,i) += rarm_fb_theta[i][j]*pow(exec_tm, j);
    }
  }
    
  // larm
  // 今は同じ値を送り続ける
  targetEEPose(3,0) = larm_pos_params[0][0];
  targetEEPose(3,1) = larm_pos_params[1][0];
  targetEEPose(3,2) = larm_pos_params[2][0];
  targetEEPose(3,3) = larm_rpy_params[0][0];
  targetEEPose(3,4) = larm_rpy_params[1][0];
  targetEEPose(3,5) = larm_rpy_params[2][0];

  /* ---------------------------------------------------------------------------------------------------- */
  
  // // 昔のver
  // // x,y = start + 0.5 * range * (1 - cos(2pi/T)*t)
  // //   z = start + 0.5 * range * (1 - cos(pi/T)*t)

  // // legは全て0を送るので何もしない

  // // rarm
  // // position
  // targetEEPose(2,0) = rarm_pos_params[0][0] + 0.5*rarm_pos_params[0][1]*(1 - cos((2*M_PI/motion_time)*exec_tm));
  // targetEEPose(2,1) = rarm_pos_params[1][0] + 0.5*rarm_pos_params[1][1]*(1 - cos((2*M_PI/motion_time)*exec_tm));
  // targetEEPose(2,2) = rarm_pos_params[2][0] + 0.5*rarm_pos_params[2][1]*(1 - cos((M_PI/motion_time)*exec_tm + M_PI));
  // // orientation
  // targetEEPose(2,3) = rarm_rpy_params[0][0] + 0.5*rarm_rpy_params[0][1]*(1 - cos((M_PI/motion_time)*exec_tm + M_PI));
  // targetEEPose(2,4) = rarm_rpy_params[1][0] + 0.5*rarm_rpy_params[1][1]*(1 - cos((M_PI/motion_time)*exec_tm + M_PI));
  // targetEEPose(2,5) = rarm_rpy_params[2][0] + 0.5*rarm_rpy_params[2][1]*(1 - cos((M_PI/motion_time)*exec_tm + M_PI));
  
  // // larm
  // // 今は同じ値を送り続ける
  // targetEEPose(3,0) = larm_pos_params[0][0];
  // targetEEPose(3,1) = larm_pos_params[1][0];
  // targetEEPose(3,2) = larm_pos_params[2][0];
  // targetEEPose(3,3) = larm_rpy_params[0][0];
  // targetEEPose(3,4) = larm_rpy_params[1][0];
  // targetEEPose(3,5) = larm_rpy_params[2][0];
}
  
void BasketballMotionController::initPose(){
  // legは全て0を送るので何もしない
  
  // rarm
  targetEEPose(2,0) = rarm_pos_params[0][0];
  targetEEPose(2,1) = rarm_pos_params[1][0];
  targetEEPose(2,2) = rarm_pos_params[2][0];
  targetEEPose(2,3) = rarm_rpy_params[0][0];
  targetEEPose(2,4) = rarm_rpy_params[1][0];
  targetEEPose(2,5) = rarm_rpy_params[2][0];
  
  // 左手
  targetEEPose(3,0) = larm_pos_params[0][0];
  targetEEPose(3,1) = larm_pos_params[1][0];
  targetEEPose(3,2) = larm_pos_params[2][0];
  targetEEPose(3,3) = larm_rpy_params[0][0];
  targetEEPose(3,4) = larm_rpy_params[1][0];
  targetEEPose(3,5) = larm_rpy_params[2][0];

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
  } else if (!startDribbleMotion_flag){
    startDribbleMotion_flag = true;
    motion_state = 1;
    std::cout << "Start DribbleMode" << std::endl;
    return true;
  } else {
    std::cout << "Dribble motion is already in progress" << std::endl;
  }
}

bool BasketballMotionController::stopDribbleMotion(){
  motionEnd_flag = true;
  // startDribbleMode_flag以外をfalse
  
  return true;
}

bool BasketballMotionController::setBasketballMotionControllerParam
(const OpenHRP::BasketballMotionControllerService::BasketballMotionControllerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);

  if (i_param.target_contact_ball_state.length() == 6) {
    targetContactBallState[0].pos.x = i_param.target_contact_ball_state[0];
    targetContactBallState[0].pos.y = i_param.target_contact_ball_state[1];
    targetContactBallState[0].pos.z = i_param.target_contact_ball_state[2];
  }

  if (i_param.hand_offset.length() == 6) {
    for (int i=0; i<hand_offset.size(); i++) {
      hand_offset[i] = i_param.hand_offset[i];
    } 
  }

  if (i_param.rarm_range.length() == 6) {
    for (int i=0; i<3; i++) {
      rarm_pos_params[i][1] = i_param.rarm_range[i];
    }
    for (int i=0; i<3; i++) {
      rarm_rpy_params[i][1] = i_param.rarm_range[i+3];
    }
  }

  if (i_param.larm_range.length() == 6) {
    for (int i=0; i<3; i++) {
      larm_pos_params[i][1] = i_param.larm_range[i];
    }
    for (int i=0; i<3; i++) {
      larm_rpy_params[i][1] = i_param.larm_range[i+3];
    } 
  }

  ff_motion_time = i_param.ff_motion_time;

  return true;
}

bool BasketballMotionController::getBasketballMotionControllerParam
(OpenHRP::BasketballMotionControllerService::BasketballMotionControllerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);

  i_param.target_contact_ball_state[0] = targetContactBallState[0].pos.x;
  
  i_param.target_contact_ball_state[1] = targetContactBallState[0].pos.y;
  i_param.target_contact_ball_state[2] = targetContactBallState[0].pos.z;
    
  for (int i=0; i<hand_offset.size(); i++) {
    i_param.hand_offset[i] = hand_offset[i];
  }
  
  for (int i=0; i<3; i++) {
    i_param.rarm_range[i] = rarm_pos_params[i][1];
  }
  for (int i=0; i<3; i++) {
    i_param.rarm_range[i+3] = rarm_rpy_params[i][1];
  }
  
  for (int i=0; i<3; i++) {
    i_param.larm_range[i] = larm_pos_params[i][1];
  }
  for (int i=0; i<3; i++) {
    i_param.larm_range[i+3] = larm_rpy_params[i][1];
  } 

  i_param.ff_motion_time = ff_motion_time;

  
  return true;
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
