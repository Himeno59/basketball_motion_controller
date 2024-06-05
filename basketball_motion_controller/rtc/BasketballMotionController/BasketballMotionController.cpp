#include "BasketballMotionController.h"

BasketballMotionController::BasketballMotionController(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_objRefIn_("objRefIn", m_objRef_),
  m_rarm_eePoseOut_("rarm_eePoseOut", m_rarm_eePose_),
  // m_eePoseIn_("EEPoseOut", m_eePose_),
  
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
}

RTC::ReturnCode_t BasketballMotionController::onInitialize(){
  addInPort("objRefIn", this->m_objRefIn_);
  addOutPort("rarm_eePoseOut", this->m_rarm_eePoseOut_);
  
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
  if (this->m_objRefIn_.isNew()){
    this->m_objRefIn_.read();
  }

  // generate targetEEPose
  this->genTargetEEPose();

  // writeOutPortData
  {
    this->m_rarm_eePoseOut_.write();
  }

  return RTC::RTC_OK;
}

// void BasketballMotionController::genTargetEEPose() const {
//   RTC::TimedPose3DSeq targetEEPose; // 0:rleg, 1:lleg, 2:rarm, 3:larm
//   // rarm
//   // point
//   targetEEPose[2].position.x = 0.0;
//   targetEEPose[2].position.y = 0.0;
//   targetEEPose[2].position.z = 0.0;
//   // orientation
//   targetEEPose[2].orientation.r = 0.0;
//   targetEEPose[2].orientation.p = 0.0;
//   targetEEPose[2].orientation.y = 0.0;
  
//   // RTC::TimedPose3D rarm_targetEEPose;
//   // RTC::TimedPose3D larm_targetEEPose;
//   // RTC::TimedPose3D rleg_targetEEPose;
//   // RTC::TimedPose3D lleg_targetEEPose;  
//   // // point
//   // ratm_targetEEPose.position.x = 0.0;
//   // ratm_targetEEPose.position.y = 0.0;
//   // ratm_targetEEPose.position.z = 0.0;
  
//   // // orientation
//   // ratm_targetEEPose.orientation.r = 0.0;
//   // ratm_targetEEPose.orientation.p = 0.0;
//   // ratm_targetEEPose.orientation.y = 0.0;
      
//   for(int i=0;i<4;i++){
//     // 両手を与えるようにするとして、脚はどうする?
//     m_eePose_[i] = targetEEPose[i];
//   }
// }

void BasketballMotionController::genTargetEEPose() {
  RTC::TimedPose3D targetEEPose;
  // point
  targetEEPose.data.position.x = 0.0;
  targetEEPose.data.position.y = 0.0;
  targetEEPose.data.position.z = 0.0;
  // orientation
  targetEEPose.data.orientation.r = 0.0;
  targetEEPose.data.orientation.p = 0.0;
  targetEEPose.data.orientation.y = 0.0;

  m_rarm_eePose_ = targetEEPose;

  std::cout << "[targetEEPose]" << std::endl;
  std::cout << "position.x: " << m_rarm_eePose_.data.position.x << std::endl;
  std::cout << "position.y: " << m_rarm_eePose_.data.position.y << std::endl;
  std::cout << "porision.z: " << m_rarm_eePose_.data.position.z << std::endl;
  std::cout << "orientation.r: " << m_rarm_eePose_.data.orientation.r << std::endl;
  std::cout << "orientation.p: " << m_rarm_eePose_.data.orientation.p << std::endl;
  std::cout << "orientation.y: " << m_rarm_eePose_.data.orientation.y << std::endl;
  
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
