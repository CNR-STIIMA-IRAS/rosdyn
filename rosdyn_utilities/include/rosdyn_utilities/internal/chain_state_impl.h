#ifndef ROSDYN_UTILITIES__CHAIN_STATUS_IMPL__H
#define ROSDYN_UTILITIES__CHAIN_STATUS_IMPL__H

#include <Eigen/QR> 
#include <sstream>
#include <rosdyn_utilities/chain_state.h>

#define SP std::fixed  << std::setprecision(5)
#define TP(X) std::fixed << std::setprecision(5) << X.format(m_cfrmt)


#include <Eigen/SVD>

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


// template<int N> 
// bool ChainStateN<N>::init(ChainInterfacePtr kin) 
// {   
//   if(!kin)
//     return false;
//   kin_ = kin;
//   this->q_.resize(this->kin_->nAx());
//   this->qd_.resize(this->kin_->nAx());
//   this->qdd_.resize(this->kin_->nAx());
//   this->effort_.resize(this->kin_->nAx());
//   this->external_effort_.resize(this->kin_->nAx());
//   thisvalue().setZero();
// } 

template<int N> 
ChainStateN<N>::ChainStateN(ChainInterfacePtr kin) 
{   
  assert(kin); 
  init(kin);
} 

template<int N>
void ChainStateN<N>::setZero()
{
  this->q_.value().setZero();
  this->qd_.value().setZero();
  this->qdd_.value().setZero();
  this->effort_.value().setZero();
  this->external_effort_.value().setZero();
  updateTransformations();
}

template<int N> 
void ChainStateN<N>::copy(const ChainStateN<N>& cpy)
{
  this->kin_ = cpy.kin_ ;
  this->q_ = cpy.q_;
  this->qd_ = cpy.qd_;
  this->qdd_ = cpy.qdd_;
  this->effort_ = cpy.effort_;
  this->external_effort_ = cpy.external_effort_;
  updateTransformations();
}

template<int N> 
ChainStateN<N>::ChainStateN(const ChainStateN<N>& cpy)
{
  copy(cpy); 
}

template<int N>
ChainStateN<N>& ChainStateN<N>::operator=(const ChainStateN<N>& rhs)
{
  copy(rhs); 
  return *this;
}

template<int N>
ChainStateN<N>& ChainStateN<N>::updateTransformations(bool effort_to_wrench)
{
  Tbt_ = kin_->getChain()->getTransformation(this->q());
  jacobian_ = kin_->getChain()->getJacobian(this->q());
  twist_= kin_->getChain()->getTwistTool(this->q(), this->qd());
  twistd_  = kin_->getChain()->getDTwistTool(this->q(), this->qd(),this->qdd());

  if(effort_to_wrench)
  {
    Eigen::Matrix<double,N,6> jt = jacobian_.transpose();
    Eigen::Matrix<double,6,N> jti;
    jti = pseudoInverse(jt);
    
    wrench_.update(jti * external_effort_.value());
  }
  else
  {
    assert(0);
  }
  
  return *this;
}


inline
void get_joint_names(ros::NodeHandle& nh, std::vector<std::string>& names)
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
        if(nh.getParam(key, joint_name))
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

}  // rosdyn

#undef TP
#undef SP

#endif  //  ROSDYN_UTILITIES__CHAIN_STATUS_IMPL__H