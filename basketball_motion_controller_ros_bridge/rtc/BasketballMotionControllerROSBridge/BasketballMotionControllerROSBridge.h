#ifndef BasketballMotionControllerROSBridge_H
#define BasketballMotionControllerROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <basketball_motion_controller_msgs/idl/BasketballMotionController.hh>

#include <ros/ros.h>

class BasketballMotionControllerROSBridge : public RTC::DataFlowComponentBase{
protected:
  ros::NodeHandle nh;

public:
  BasketballMotionControllerROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};

extern "C"
{
  void BasketballMotionControllerROSBridgeInit(RTC::Manager* manager);
};

#endif // BasketballMotionControllerROSBridge_H
