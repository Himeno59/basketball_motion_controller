#include "BasketballMotionControllerService_impl.h"
#include "BasketballMotionController.h"

BasketballMotionControllerService_impl::BasketballMotionControllerService_impl()
{
}

BasketballMotionControllerService_impl::~BasketballMotionControllerService_impl()
{
}

CORBA::Boolean BasketballMotionControllerService_impl::startDribbleMode()
{
  return this->comp_->startDribbleMode();
};

CORBA::Boolean BasketballMotionControllerService_impl::stopDribbleMode()
{
  return this->comp_->stopDribbleMode();
};

CORBA::Boolean BasketballMotionControllerService_impl::startDribbleMotion()
{
  return this->comp_->startDribbleMotion();
};

CORBA::Boolean BasketballMotionControllerService_impl::stopDribbleMotion()
{
  return this->comp_->stopDribbleMotion();
};

void BasketballMotionControllerService_impl::setComp(BasketballMotionController *i_comp)
{
  comp_ = i_comp;
}

CORBA::Boolean BasketballMotionControllerService_impl::BasketballMotionControllerParam(const CORBA::Double data)
{
  return comp_->BasketballMotionControllerParam(data);
};
