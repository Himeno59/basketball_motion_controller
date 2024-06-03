#include "BasketballMotionControllerService_impl.h"
#include "BasketballMotionController.h"

BasketballMotionControllerService_impl::BasketballMotionControllerService_impl()
{
}

BasketballMotionControllerService_impl::~BasketballMotionControllerService_impl()
{
}

void BasketballMotionControllerService_impl::setComp(BasketballMotionController *i_comp)
{
  comp_ = i_comp;
}

CORBA::Boolean BasketballMotionControllerService_impl::basketballmotionParam(const CORBA::Double data)
{
  return comp_->basketballmotionParam(data);
};
