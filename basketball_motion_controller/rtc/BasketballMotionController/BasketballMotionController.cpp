#include "BasketballMotionController.h"

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
  m_basketballmotionControllerServicePort_("BasketballMotionControllerService")
{
  this->m_service0_.setComp(this);

  loop = 0;
  dt = 0.002; // [sec]
  exec_tm = 0.0; // [sec]
  motion_time = 1.0; // [sec]
  pos_range = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}}; // x,y,z : high,low [m]
  pos_range = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}}; // r,p,y : high.low [rad]
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
  
  // addInPort("qRefIn", this->m_qRefIn_);
  // addInPort("tauRefIn", this->m_tauRefIn_);
  // addInPort("qActIn", this->m_qActIn_);
  // addInPort("dqActIn", this->m_dqActIn_);
  // addInPort("tauActIn", this->m_tauActIn_);
  // addOutPort("q", this->m_qOut_);
  // addOutPort("tauOut", this->m_tauOut_);
  this->m_basketballmotionControllerServicePort_.registerProvider("service0", "BasketballMotionControllerService", this->m_service0_);
  addPort(this->m_basketballmotionControllerServicePort_);

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

  // readInPortData
  if (m_objRefIn_.isNew()){
    m_objRefIn_.read();
  }

  // generate targetEEPose
  genTargetEEPose();

  // writeOutPortData
  {
    for(int i=0;i<4;i++){
      m_eePoseOut_[i]->write();
    }
  }

  return RTC::RTC_OK;
}

void BasketballMotionController::genTargetEEPose() {
  for(int i=0;i<4;i++){
    RTC::TimedPose3D targetEEPose;
    if(i=2){
      // 右手
      // position
      targetEEPose.data.position.x = 0.0;
      targetEEPose.data.position.y = 0.0;
      targetEEPose.data.position.z = 0.0;
      // orientation
      targetEEPose.data.orientation.r = 0.0;
      targetEEPose.data.orientation.p = 0.0;
      targetEEPose.data.orientation.y = 0.0;
    } else if(i=3) {
      // 左手
      // position
      targetEEPose.data.position.x = 0.0;
      targetEEPose.data.position.y = 0.0;
      targetEEPose.data.position.z = 0.0;
      // orientation
      targetEEPose.data.orientation.r = 0.0;
      targetEEPose.data.orientation.p = 0.0;
      targetEEPose.data.orientation.y = 0.0;
    } else {
      // 脚は全て0を送る
      // position
      targetEEPose.data.position.x = 0.0;
      targetEEPose.data.position.y = 0.0;
      targetEEPose.data.position.z = 0.0;
      // orientation
      targetEEPose.data.orientation.r = 0.0;
      targetEEPose.data.orientation.p = 0.0;
      targetEEPose.data.orientation.y = 0.0;
    }
          
    m_eePose_[i] = targetEEPose;
  }
  
  // std::cout << "[targetEEPose]" << std::endl;
  // std::cout << "position.x: " << m_eePose_[2].data.position.x << std::endl;
  // std::cout << "position.y: " << m_eePose_[2].data.position.y << std::endl;
  // std::cout << "porision.z: " << m_eePose_[2].data.position.z << std::endl;
  // std::cout << "orientation.r: " << m_eePose_[2].data.orientation.r << std::endl;
  // std::cout << "orientation.p: " << m_eePose_[2].data.orientation.p << std::endl;
  // std::cout << "orientation.y: " << m_eePose_[2].data.orientation.y << std::endl;
  
}

bool BasketballMotionController::basketballmotionParam(const double data){
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
