// -*-C++-*-
#ifndef BasketballMotionControllerSERVICESVC_IMPL_H
#define BasketballMotionControllerSERVICESVC_IMPL_H

#include "basketball_motion_controller/idl/BasketballMotionControllerService.hh"

class BasketballMotionController;

class BasketballMotionControllerService_impl
  : public virtual POA_OpenHRP::BasketballMotionControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  BasketballMotionControllerService_impl();
  ~BasketballMotionControllerService_impl();
  CORBA::Boolean startDribbleMode();
  CORBA::Boolean stopDribbleMode();
  CORBA::Boolean startDribbleMotion();
  CORBA::Boolean stopDribbleMotion();
  
  CORBA::Boolean setBasketballMotionControllerParam(const OpenHRP::BasketballMotionControllerService::BasketballMotionControllerParam& i_param);
  CORBA::Boolean getBasketballMotionControllerParam(OpenHRP::BasketballMotionControllerService::BasketballMotionControllerParam_out i_param);
  void setComp(BasketballMotionController *i_comp);
private:
  BasketballMotionController *comp_;
};

#endif
