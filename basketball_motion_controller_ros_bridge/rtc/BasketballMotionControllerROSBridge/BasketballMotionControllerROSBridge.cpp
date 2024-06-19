#include "BasketballMotionControllerROSBridge.h"

BasketballMotionControllerROSBridge::BasketballMotionControllerROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_nowObjStateOut_("nowObjStateOut", m_nowObjState_),
  m_predObjStateOut_("predObjStateOut", m_predObjState_)
{
  // 
}

RTC::ReturnCode_t BasketballMotionControllerROSBridge::onInitialize(){
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh("");
  
  addOutPort("nowObjStateOut", this->m_nowObjStateOut_);
  addOutPort("predObjStateOut", this->m_predObjStateOut_);

  // now_state_sub_ = nh.subscribe("/ObjectTrajectoryEstimator/now_ball_state", 1, &BasketballMotionControllerROSBridge::nowStateCb, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
  // pred_state_sub_ = nh.subscribe("/ObjectTrajectoryEstimator/pred_ball_state", 1, &BasketballMotionControllerROSBridge::predStateCb, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
  now_state_sub_ = pnh.subscribe("now_state_input", 1, &BasketballMotionControllerROSBridge::nowStateCb, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());
  pred_state_sub_ = pnh.subscribe("pred_state_input", 1, &BasketballMotionControllerROSBridge::predStateCb, this, ros::TransportHints().unreliable().reliable().tcpNoDelay());

  // pub_ = pnh.advertise<~>("output", 1);
  
  return RTC::RTC_OK;
}

RTC::ReturnCode_t BasketballMotionControllerROSBridge::onExecute(RTC::UniqueId ec_id){
  
  ros::spinOnce();

  return RTC::RTC_OK;
}

void BasketballMotionControllerROSBridge::nowStateCb(object_trajectory_estimator::BallStateStamped::ConstPtr msg) {
  // pos
  m_nowObjState_.pos.x = msg->pos.x;
  m_nowObjState_.pos.y = msg->pos.y;
  m_nowObjState_.pos.z = msg->pos.z;

  // vel
  m_nowObjState_.vel.x = msg->vel.x;
  m_nowObjState_.vel.y = msg->vel.y;
  m_nowObjState_.vel.z = msg->vel.z;

  // tm
  m_nowObjState_.tm.sec = msg->header.stamp.sec;
  m_nowObjState_.tm.nsec = msg->header.stamp.nsec;
  
  m_nowObjStateOut_.write();
}

void BasketballMotionControllerROSBridge::predStateCb(object_trajectory_estimator::BallStateStamped::ConstPtr msg) {

  // pos
  m_predObjState_.pos.x = msg->pos.x;
  m_predObjState_.pos.y = msg->pos.y;
  m_predObjState_.pos.z = msg->pos.z;

  // vel
  m_predObjState_.vel.x = msg->vel.x;
  m_predObjState_.vel.y = msg->vel.y;
  m_predObjState_.vel.z = msg->vel.z;

  // tm
  m_predObjState_.tm.sec = msg->header.stamp.sec;
  m_predObjState_.tm.nsec = msg->header.stamp.nsec;
  
  m_predObjStateOut_.write();
  
}

static const char* BasketballMotionControllerROSBridge_spec[] = {
  "implementation_id", "BasketballMotionControllerROSBridge",
  "type_name",         "BasketballMotionControllerROSBridge",
  "description",       "BasketballMotionControllerROSBridge component",
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
    void BasketballMotionControllerROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(BasketballMotionControllerROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<BasketballMotionControllerROSBridge>, RTC::Delete<BasketballMotionControllerROSBridge>);
    }
};
