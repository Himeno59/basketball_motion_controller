#ifndef BasketballMotionControllerROSBridge_H
#define BasketballMotionControllerROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <basketball_motion_controller_msgs/idl/BasketballMotionController.hh>

#include <ros/ros.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "object_trajectory_estimator/BallStateStamped.h"

class BasketballMotionControllerROSBridge : public RTC::DataFlowComponentBase{
protected:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  // RTMからROSにvisualizationしたい場合に使う
  /* RTC::TimedDoubleSeq m_RTM_; */
  /* RTC::InPort<RTC::TimedDoubleSeq> m_In_; */
  /* ros::Publisher pub_; */

  // ros -> rtm //
  // now state
  basketball_motion_controller_msgs::ObjStateStamped m_nowObjState_;
  RTC::OutPort<basketball_motion_controller_msgs::ObjStateStamped> m_nowObjStateOut_;
  ros::Subscriber now_state_sub_;
  // pred state
  basketball_motion_controller_msgs::ObjStateStamped m_predObjState_;
  RTC::OutPort<basketball_motion_controller_msgs::ObjStateStamped> m_predObjStateOut_;
  ros::Subscriber pred_state_sub_;
  /* // fb flag */
  /* RTC::boolean m_fb_; */
  /* RTC::OutPort<RTC::boolean> m_fbOut_; */
  /* ros::Subscriber sub3_; */
 
  void nowStateCb(object_trajectory_estimator::BallStateStamped::ConstPtr msg);
  void predStateCb(object_trajectory_estimator::BallStateStamped::ConstPtr msg);

public:
  BasketballMotionControllerROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

private:
  double m_dt;
  double m_prev_time;
  bool fb_flag;
};

extern "C"
{
  void BasketballMotionControllerROSBridgeInit(RTC::Manager* manager);
};

#endif // BasketballMotionControllerROSBridge_H
