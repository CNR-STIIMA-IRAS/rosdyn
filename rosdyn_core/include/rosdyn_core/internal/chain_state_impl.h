#ifndef ROSDYN_CORE__CHAIN_STATUS_IMPL__H
#define ROSDYN_CORE__CHAIN_STATUS_IMPL__H

#include <sstream>
#include <rosdyn_core/chain_state.h>

#define SP std::fixed  << std::setprecision(5)
#define TP(X) std::fixed << std::setprecision(5) << X.format(m_cfrmt)

namespace rosdyn
{

inline 
ChainState::ChainState(ChainInterfacePtr kin) 
: kin_(kin) 
{   
  assert(kin); 
  this->q_     .resize(this->kin_->nAx());
  this->qd_    .resize(this->kin_->nAx());
  this->qdd_   .resize(this->kin_->nAx());
  this->effort_.resize(this->kin_->nAx());
  this->setZero();
} 

inline 
void ChainState::setZero()
{
  this->q_     .setZero();
  this->qd_    .setZero();
  this->qdd_   .setZero();
  this->effort_.setZero();
  updateTransformation();
}

inline 
ChainState::ChainState(const ChainState& cpy)
{
  this->kin_         = cpy.kin_   ;
  this->q_           = cpy.q_     ;
  this->qd_          = cpy.qd_    ;
  this->qdd_         = cpy.qdd_   ;
  this->effort_      = cpy.effort_;
  updateTransformation();
  
}

inline
ChainState& ChainState::operator=(const ChainState& rhs)
{
  this->kin_         = rhs.kin_   ;
  this->q_           = rhs.q_     ;
  this->qd_          = rhs.qd_    ;
  this->qdd_         = rhs.qdd_   ;
  this->effort_      = rhs.effort_;
  updateTransformation();
  return *this;
}

inline
ChainState& ChainState::updateTransformation( )
{
  Tbt_      = kin_->getChain()->getTransformation(this->q());
  jacobian_ = kin_->getChain()->getJacobian(this->q());
  twist_    = kin_->getChain()->getTwistTool(this->q(), this->qd());
  twistd_   = kin_->getChain()->getDTwistTool(this->q(), this->qd(),this->qdd());
  
  return *this;
}


}  // rosdyn

#undef TP
#undef SP

#endif  //  ROSDYN_CORE__CHAIN_STATUS_IMPL__H