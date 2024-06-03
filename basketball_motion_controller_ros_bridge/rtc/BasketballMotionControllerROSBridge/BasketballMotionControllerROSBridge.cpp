#include "BasketballMotionControllerROSBridge.h"

BasketballMotionControllerROSBridge::BasketballMotionControllerROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager)
{
}

RTC::ReturnCode_t BasketballMotionControllerROSBridge::onInitialize(){
  ros::NodeHandle pnh("~");
  
  return RTC::RTC_OK;
}

RTC::ReturnCode_t BasketballMotionControllerROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  return RTC::RTC_OK;
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
