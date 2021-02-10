#pragma once

#ifndef ROSDYN_UTILITIES__CHAIN_STATE_IMPL__H
#define ROSDYN_UTILITIES__CHAIN_STATE_IMPL__H

#include <Eigen/QR>
#include <Eigen/SVD>
#include <sstream>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <rosdyn_utilities/chain_state.h>

#define SP std::fixed  << std::setprecision(5)
#define TP(X) std::fixed << std::setprecision(5) << X.format(m_cfrmt)


template<typename MatType>
using PseudoInverseType = Eigen::Matrix<typename MatType::Scalar, MatType::ColsAtCompileTime, MatType::RowsAtCompileTime>;

template<typename MatType>
PseudoInverseType<MatType> pseudoInverse(const MatType &a, double epsilon = std::numeric_limits<double>::epsilon())
{
  using WorkingMatType = Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0,
                                       MatType::MaxRowsAtCompileTime, MatType::MaxColsAtCompileTime>;
  Eigen::BDCSVD<WorkingMatType> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
  svd.setThreshold(epsilon*std::max(a.cols(), a.rows()));
  Eigen::Index rank = svd.rank();
  Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, MatType::RowsAtCompileTime,
                0, Eigen::BDCSVD<WorkingMatType>::MaxDiagSizeAtCompileTime, MatType::MaxRowsAtCompileTime>
    tmp = svd.matrixU().leftCols(rank).adjoint();
  tmp = svd.singularValues().head(rank).asDiagonal().inverse() * tmp;
  return svd.matrixV().leftCols(rank) * tmp;
}


namespace rosdyn
{

template<int N, int MN>
inline ChainState<N,MN>::ChainState(ChainPtr kin)
  : ChainState<N,MN>::ChainState(*kin)
{
}

template<int N, int MN>
inline ChainState<N,MN>::ChainState(Chain& kin)
{
  if(!init(kin))
  {
    throw std::runtime_error( std::string(__PRETTY_FUNCTION__)+":"+std::to_string(__LINE__)+":"
            + std::string(" Error in the init() function. The number of joints of the input qrgument ChainPtr is ")
                              +std::to_string(kin.getActiveJointsNumber())+", while the "
          );
  }

}


template<int N, int MN>
inline bool ChainState<N,MN>::init(ChainPtr kin)
{
  /*size_t l = __LINE__;
  try
  {
    l = __LINE__;
    if(!kin)
    {
      return false;
    }
    l = __LINE__;
    if(int(kin->getActiveJointsNumber())!=N)
    {
      return false;
    }
    l = __LINE__;
    //kin_ = kin;
    joint_names_ = kin->getActiveJointsName();
    l = __LINE__;
    this->setZero(kin);
  }
  catch(std::exception& e)
  {
    std::cerr <<  __PRETTY_FUNCTION__ << ":" << __LINE__ << ": " ;
    std::cerr << "Exception at line: "
              << std::to_string(l) << " error: " + std::string(e.what())
              << std::endl;
  }*/
  return init(*kin);
}

template<int N, int MN>
inline bool ChainState<N,MN>::init(Chain& kin)
{
  size_t l = __LINE__;
  try
  {
    if(kin.getActiveJointsNumber()<=0)
    {
      std::cerr <<  __PRETTY_FUNCTION__ << ":" << __LINE__ << ": " ;
      std::cerr << " The Chain is 0-dof" << std::endl;
      return false;
    }
    l = __LINE__;
    if(N==-1)
    {
      this->jacobian_.resize(6,kin.getActiveJointsNumber());
    }

    if(!this->q_.resize(kin.getActiveJointsNumber()) || !this->qd_.resize(kin.getActiveJointsNumber())
      || !this->qdd_.resize(kin.getActiveJointsNumber()) || !this->external_effort_.resize(kin.getActiveJointsNumber())
        || !this->effort_.resize(kin.getActiveJointsNumber()) || this->jacobian_.cols() != kin.getActiveJointsNumber())
    {
      return false;
    }
    l = __LINE__;
    //kin_ = kin;
    joint_names_ = kin.getActiveJointsName();
    l = __LINE__;
    this->setZero(kin);
  }
  catch(std::exception& e)
  {
    std::cerr <<  __PRETTY_FUNCTION__ << ":" << __LINE__ << ": " ;
    std::cerr << "Exception at line: "
              << std::to_string(l) << " error: " + std::string(e.what())
              << std::endl;
    return false;
  }
  return true;
}

template<int N, int MN>
inline void ChainState<N,MN>::setZero(ChainPtr kin)
{
  eigen_utils::setZero(this->q_.value());
  eigen_utils::setZero(this->qd_.value());
  eigen_utils::setZero(this->qdd_.value());
  eigen_utils::setZero(this->effort_.value());
  eigen_utils::setZero(this->external_effort_.value());
  if(kin)
  {
    this->updateTransformations(kin, SECOND_ORDER|FFWD_STATIC);
  }
}

template<int N, int MN>
inline void ChainState<N,MN>::setZero(Chain& kin)
{
  eigen_utils::setZero(this->q_.value());
  eigen_utils::setZero(this->qd_.value());
  eigen_utils::setZero(this->qdd_.value());
  eigen_utils::setZero(this->effort_.value());
  eigen_utils::setZero(this->external_effort_.value());
  this->updateTransformations(kin, SECOND_ORDER|FFWD_STATIC);
}

template<int N, int MN>
inline void ChainState<N,MN>::copy(const ChainState<N,MN>& cpy, CopyType what)
{
  //this->kin_ = cpy.kin_ ;
  if(what ==this->ONLY_JOINT || what == this->FULL_STATE)
  {
    this->q_ = cpy.q_;
    this->qd_ = cpy.qd_;
    this->qdd_ = cpy.qdd_;
    this->effort_ = cpy.effort_;
    this->external_effort_ = cpy.external_effort_;
  }
  if(what ==this->ONLY_CART || what == this->FULL_STATE)
  {
    this->Tbt_      = cpy.Tbt_;
    this->twist_    = cpy.twist_;
    this->twistd_   = cpy.twistd_;
    this->wrench_   = cpy.wrench_;
    this->jacobian_ = cpy.jacobian_;
  }
}

template<int N, int MN>
template<int n, std::enable_if_t<n==1,int> >
inline ChainState<N,MN>& ChainState<N,MN>::updateTransformations(ChainPtr kin, int ffwd_kin_type)
{
  Eigen::Matrix<double,1,1> _q,_qd,_qdd,_external_effort;
  if(!kin)
  {
    throw std::runtime_error("kin nullptr!Chain State has not the ownership of the ChainInterface*!");
  }
  eigen_utils::copy(_q,q_.value());
  eigen_utils::copy(_qd,qd_.value());
  eigen_utils::copy(_qdd,qdd_.value());
  eigen_utils::copy(_external_effort,external_effort_.value());

  if(ffwd_kin_type & ZERO_ORDER)
  {
    Tbt_ = kin->getTransformation(_q);
  }
  if(ffwd_kin_type & FIRST_ORDER)
  {
    jacobian_ = kin->getJacobian(_qd);
    twist_    = kin->getTwistTool(_q,_qd);
  }
  if(ffwd_kin_type & SECOND_ORDER)
  {
    twistd_   = kin->getDTwistTool(_q,_qd,_qdd);
  }

  if(ffwd_kin_type & FFWD_STATIC)
  {
//    Eigen::Matrix<double,1,6> jt = jacobian_.transpose();
//    Eigen::Matrix<double,6,1> jti;
//    jti = pseudoInverse(jt);
    wrench_.update( jacobian_ * _external_effort );
  }
  else if(ffwd_kin_type & INV_STATIC)
  {
    assert(0);
  }
  return *this;
}

template<int N, int MN>
template<int n, std::enable_if_t<n!=1,int> >
inline ChainState<N,MN>& ChainState<N,MN>::updateTransformations(ChainPtr kin, int ffwd_kin_type)
{
  if(!kin)
  {
    throw std::runtime_error("kin nullptr!Chain State has not the ownership of the ChainInterface*!");
  }

  if(ffwd_kin_type & ZERO_ORDER)
  {
    Tbt_ = kin->getTransformation(q_.value());
  }
  if(ffwd_kin_type & FIRST_ORDER)
  {
    jacobian_ = kin->getJacobian(qd_.value());
    twist_    = kin->getTwistTool(q_.value(),qd_.value());
  }
  if(ffwd_kin_type & SECOND_ORDER)
  {
    twistd_   = kin->getDTwistTool(q_.value(),qd_.value(),qdd_.value());
  }

  if(ffwd_kin_type & FFWD_STATIC)
  {
    Eigen::Matrix<double,N,6> jt = jacobian_.transpose();
    Eigen::Matrix<double,6,N> jti;
    jti = pseudoInverse(jt);
    wrench_.update(jti * external_effort_.value());
  }
  else if(ffwd_kin_type & INV_STATIC)
  {
    assert(0);
  }
  return *this;
}


template<int N, int MN>
template<int n, std::enable_if_t<n==1,int> >
inline ChainState<N,MN>& ChainState<N,MN>::updateTransformations(Chain& kin, int ffwd_kin_type)
{
  Eigen::Matrix<double,1,1> _q,_qd,_qdd,_external_effort;
  eigen_utils::copy(_q,q_.value());
  eigen_utils::copy(_qd,qd_.value());
  eigen_utils::copy(_qdd,qdd_.value());
  eigen_utils::copy(_external_effort,external_effort_.value());

  if(ffwd_kin_type & ZERO_ORDER)
  {
    Tbt_ = kin.getTransformation(_q);
  }
  if(ffwd_kin_type & FIRST_ORDER)
  {
    jacobian_ = kin.getJacobian(_qd);
    twist_    = kin.getTwistTool(_q,_qd);
  }
  if(ffwd_kin_type & SECOND_ORDER)
  {
    twistd_   = kin.getDTwistTool(_q,_qd,_qdd);
  }

  if(ffwd_kin_type & FFWD_STATIC)
  {
//    Eigen::Matrix<double,1,6> jt = jacobian_.transpose();
//    Eigen::Matrix<double,6,1> jti;
//    jti = pseudoInverse(jt);
    wrench_.update( jacobian_ * _external_effort );
  }
  else if(ffwd_kin_type & INV_STATIC)
  {
    assert(0);
  }
  return *this;
}

template<int N, int MN>
template<int n, std::enable_if_t<n!=1,int> >
inline ChainState<N,MN>& ChainState<N,MN>::updateTransformations(Chain& kin, int ffwd_kin_type)
{
  if(ffwd_kin_type & ZERO_ORDER)
  {
    Tbt_ = kin.getTransformation(q_.value());
  }
  if(ffwd_kin_type & FIRST_ORDER)
  {
    jacobian_ = kin.getJacobian(qd_.value());
    twist_    = kin.getTwistTool(q_.value(),qd_.value());
  }
  if(ffwd_kin_type & SECOND_ORDER)
  {
    twistd_   = kin.getDTwistTool(q_.value(),qd_.value(),qdd_.value());
  }

  if(ffwd_kin_type & FFWD_STATIC)
  {
    Eigen::Matrix<double,N,6> jt = jacobian_.transpose();
    Eigen::Matrix<double,6,N> jti;
    jti = pseudoInverse(jt);
    wrench_.update(jti * external_effort_.value());
  }
  else if(ffwd_kin_type & INV_STATIC)
  {
    assert(0);
  }
  return *this;
}



inline void get_joint_names(ros::NodeHandle& nh, std::vector<std::string>& names)
{
  std::vector<std::string> alternative_keys =
    { "controlled_resources", "controlled_resource",
      "controlled_joints", "controlled_joint",
      "joint_names", "joint_name",
      "joint_resource/joint_names", "joint_resource/joint_name"
      "joint_resource/controlled_joints", "joint_resource/controlled_joint"
      "joint_resource/controlled_resources", "joint_resource/controlled_resource"
      };

  names.clear();
  for(auto const & key : alternative_keys)
  {
    bool ok = false;
    try
    {
      ok = nh.getParam(key, names);
    }
    catch(const std::exception& e)
    {
      std::cerr <<"Exception in getting '"<< nh.getNamespace() << "/" << key << "': " << e.what() << '\n';
      ok = false;
    }

    if(!ok)
    {
      try
      {
        std::string joint_name;
        ok = nh.getParam(key, joint_name);
        if(ok)
        {
          names.push_back(joint_name);
        }
      }
      catch(const std::exception& e)
      {
        std::cerr <<"Exception in getting '"<< nh.getNamespace() << "/" << key << "': " << e.what() << '\n';
        ok = false;
      }
    }

    if(ok)
    {
      break;
    }
  }
  return;
}

}// rosdyn

#undef TP
#undef SP

#endif  //  ROSDYN_UTILITIES__CHAIN_STATE_IMPL__H
