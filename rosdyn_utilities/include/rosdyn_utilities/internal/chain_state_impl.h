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
inline ChainState<N,MN>::ChainState(ChainInterfacePtr kin)
{
  assert(kin);
  init(kin);
}



template<int N, int MN>
inline bool ChainState<N,MN>::init(ChainInterfacePtr kin)
{
  if(!kin)
  {
    return false;
  }
  if(kin->nAx()!=N)
  {
    return false;
  }
  kin_ = kin;
  this->setZero();
  return true;
}

template<int N, int MN>
inline void ChainState<N,MN>::setZero()
{
  eigen_utils::setZero(this->q_.value());
  eigen_utils::setZero(this->qd_.value());
  eigen_utils::setZero(this->qdd_.value());
  eigen_utils::setZero(this->effort_.value());
  eigen_utils::setZero(this->external_effort_.value());
  updateTransformations();
}

template<int N, int MN>
inline void ChainState<N,MN>::copy(const ChainState<N,MN>& cpy, bool update_transform)
{
  this->kin_ = cpy.kin_ ;
  this->q_ = cpy.q_;
  this->qd_ = cpy.qd_;
  this->qdd_ = cpy.qdd_;
  this->effort_ = cpy.effort_;
  this->external_effort_ = cpy.external_effort_;
  if(update_transform)
    updateTransformations();
}

template<int N, int MN>
inline ChainState<N,MN>::ChainState(const ChainState<N,MN>& cpy)
{
  copy(cpy);
}

template<int N, int MN>
inline ChainState<N,MN>& ChainState<N,MN>::operator=(const ChainState<N,MN>& rhs)
{
  copy(rhs);
  return *this;
}

template<int N, int MN>
template<int n, std::enable_if_t<n==1,int> >
inline ChainState<N,MN>& ChainState<N,MN>::updateTransformations( const ChainState<N,MN>::Value* q,
                                                                  const ChainState<N,MN>::Value* qd,
                                                                  const ChainState<N,MN>::Value* qdd,
                                                                  const ChainState<N,MN>::Value* external_effort,
                                                                  const Eigen::Matrix<double,6,1>* wrench)
{
  static Eigen::VectorXd _q(1),_qd(1),_qdd(1),_external_effort(1);
  _q(0) = *q;
  _qd(0) = *qd;
  _qdd(0) = *qdd;
  _external_effort(0) = *external_effort;

  Tbt_      = kin_->getChain()->getTransformation(_q);
  jacobian_ = kin_->getChain()->getJacobian(_qd);
  twist_    = kin_->getChain()->getTwistTool(_q,_qd);
  twistd_   = kin_->getChain()->getDTwistTool(_q,_qd,_qdd);

  if(external_effort)
  {
    wrench_.update(jacobian_ * _external_effort);
  }
  else if(wrench)
  {
    assert(0);
  }
  return *this;
}

template<int N, int MN>
template<int n, std::enable_if_t<n!=1,int> >
inline ChainState<N,MN>& ChainState<N,MN>::updateTransformations( const ChainState<N,MN>::Value* q,
                                                                  const ChainState<N,MN>::Value* qd,
                                                                  const ChainState<N,MN>::Value* qdd,
                                                                  const ChainState<N,MN>::Value* external_effort,
                                                                  const Eigen::Matrix<double,6,1>* wrench)
{
  Tbt_      = kin_->getChain()->getTransformation(*q);
  jacobian_ = kin_->getChain()->getJacobian(*qd);
  twist_    = kin_->getChain()->getTwistTool(*q,*qd);
  twistd_   = kin_->getChain()->getDTwistTool(*q,*qd,*qdd);

  if(external_effort)
  {
    Eigen::Matrix<double,N,6> jt = jacobian_.transpose();
    Eigen::Matrix<double,6,N> jti;
    jti = pseudoInverse(jt);
    wrench_.update(jti * (*external_effort));
  }
  else if(wrench)
  {
    assert(0);
  }
  return *this;
}

template<int N, int MN>
inline ChainState<N,MN>& ChainState<N,MN>::updateTransformations(bool effort_to_wrench)
{
  if(effort_to_wrench)
  {
    return updateTransformations(&this->q(),&this->qd(), &this->qdd(), &this->external_effort(), nullptr);
  }
  else
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