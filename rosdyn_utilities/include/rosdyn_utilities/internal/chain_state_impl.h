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
  std::cout << "POIUYT: " << this->q().transpose() << std::endl;
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

template<int N>
bool ChainStateN<N>:: init(ChainInterfacePtr kin)
{
  if(!kin)
  {
    return false; 
  }
  if(kin->nAx() !=N)
  {
    return false;
  }
  kin_ = kin;
  this->setZero();
  return true;
}



/**
 * 
 */
template<>
class ChainStateN<-1>
{
protected:
  rosdyn::FilteredValue<-1> q_;
  rosdyn::FilteredValue<-1> qd_;
  rosdyn::FilteredValue<-1> qdd_;
  rosdyn::FilteredValue<-1> effort_;
  rosdyn::FilteredValue<-1> external_effort_;
  
  Eigen::Affine3d Tbt_;
  Eigen::Vector6d twist_;
  Eigen::Vector6d twistd_;
  rosdyn::FilteredValue<6> wrench_;
  Eigen::Matrix<double,6,-1> jacobian_;

  int index(const std::string& name) const
  {
    auto it = std::find(kin_->jointNames().begin(), kin_->jointNames().end(), name);
    return(it == kin_->jointNames().end()) ? -1 : std::distance(kin_->jointNames().begin(),it);
  }

  rosdyn::ChainInterfacePtr kin_;

public:

  typedef std::shared_ptr<ChainStateN> Ptr;
  typedef std::shared_ptr<ChainStateN const> ConstPtr;

  // GETTER
  const std::string& jointName(const size_t iAx) const { return kin_->jointNames().at(iAx);}
  const std::vector<std::string>& jointNames() const { return kin_->jointNames();}
  const size_t nAx() const { return kin_->nAx(); }
  const Eigen::Matrix<double,-1,1>& q() const { return q_.value();  }
  const Eigen::Matrix<double,-1,1>& qd() const { return qd_.value(); }
  const Eigen::Matrix<double,-1,1>& qdd() const { return qdd_.value();}
  const Eigen::Matrix<double,-1,1>& effort() const { return effort_.value(); }
  const Eigen::Matrix<double,-1,1>& external_effort() const { return external_effort_.value(); }

  double q(const size_t& iAx) const {CHECK_iAx(iAx); return q_.value(iAx); }
  double qd(const size_t& iAx) const {CHECK_iAx(iAx); return qd_.value(iAx); }
  double qdd(const size_t& iAx) const {CHECK_iAx(iAx); return qdd_.value(iAx); }
  double effort(const size_t& iAx) const {CHECK_iAx(iAx); return effort_.value(iAx); }
  double external_effort(const size_t& iAx) const {CHECK_iAx(iAx); return external_effort_.value(iAx); }

  double q(const std::string& name) const {DEF_iAX(name); return q(iAx); }
  double qd(const std::string& name) const {DEF_iAX(name); return qd(iAx); }
  double qdd(const std::string& name) const {DEF_iAX(name); return qdd(iAx); }
  double effort(const std::string& name) const {DEF_iAX(name); return effort(iAx); }
  double external_effort(const std::string& name) const {DEF_iAX(name); return external_effort(iAx); }

  const Eigen::Affine3d& toolPose( ) const { return Tbt_;   }
  const Eigen::Vector6d& twist( ) const { return twist_; }
  const Eigen::Vector6d& twistd( ) const { return twistd_;}
  const Eigen::Vector6d& wrench( ) const { return wrench_.value();}
  const Eigen::Matrix<double,6,-1>& jacobian( ) const { return jacobian_;  }
  
  // SETTER
  Eigen::Matrix<double,-1,1>& q() { return q_.value();  }
  Eigen::Matrix<double,-1,1>& qd() { return qd_.value(); }
  Eigen::Matrix<double,-1,1>& qdd() { return qdd_.value();}
  Eigen::Matrix<double,-1,1>& effort() { return effort_.value(); }
  Eigen::Matrix<double,-1,1>& external_effort() { return external_effort_.value(); }
  Eigen::Vector6d& wrench( ) { return wrench_.value();}

  double& q(const size_t& iAx) { return q_.value(iAx); }
  double& qd(const size_t& iAx) { return qd_.value(iAx); }
  double& qdd(const size_t& iAx) { return qdd_.value(iAx); }
  double& effort(const size_t& iAx) { return effort_.value(iAx); }
  double& external_effort(const size_t& iAx) { return external_effort_.value(iAx); }

  double& q(const std::string& name) {DEF_iAX(name); return q(iAx); }
  double& qd(const std::string& name) {DEF_iAX(name); return qd(iAx); }
  double& qdd(const std::string& name) {DEF_iAX(name); return qdd(iAx); }
  double& effort(const std::string& name) {DEF_iAX(name); return effort(iAx); }
  double& external_effort(const std::string& name) {DEF_iAX(name); return external_effort(iAx); }

  rosdyn::FilteredValue<-1>& qFilteredValue() {return q_;}
  rosdyn::FilteredValue<-1>& qdFilteredValue() {return qd_;}
  rosdyn::FilteredValue<-1>& qddFilteredValue() {return qdd_;}
  rosdyn::FilteredValue<-1>& effortFilteredValue() {return effort_;}
  rosdyn::FilteredValue<-1>& externalEffortFilteredValue() {return external_effort_;}
  rosdyn::FilteredValue<6>& wrenchFilteredValue() {return wrench_;}

  // METHODS
  ChainInterfacePtr getKin() const { return kin_; }
  ChainStateN& updateTransformations( bool effort_to_wrench = true )
  {
    Tbt_ = kin_->getChain()->getTransformation(this->q());
    jacobian_ = kin_->getChain()->getJacobian(this->q());
    twist_= kin_->getChain()->getTwistTool(this->q(), this->qd());
    twistd_  = kin_->getChain()->getDTwistTool(this->q(), this->qd(),this->qdd());

    if(effort_to_wrench)
    {
      Eigen::Matrix<double,-1,6> jt = jacobian_.transpose();
      Eigen::Matrix<double,6,-1> jti;
      jti = pseudoInverse(jt);
      
      wrench_.update(jti * external_effort_.value());
    }
    else
    {
      assert(0);
    }
    
    return *this;
  }
  
  double* handle_to_q(const size_t& iAx) {CHECK_iAx(iAx); return q_.data(iAx); }
  double* handle_to_qd(const size_t& iAx) {CHECK_iAx(iAx); return qd_.data(iAx); }
  double* handle_to_qdd(const size_t& iAx) {CHECK_iAx(iAx); return qdd_.data(iAx); }
  double* handle_to_effort(const size_t& iAx) {CHECK_iAx(iAx); return effort_.data(iAx); }
  double* handle_to_external_effort(const size_t& iAx) {CHECK_iAx(iAx); return external_effort_.data(iAx); }

  double* handle_to_q(const std::string& name) {DEF_iAX(name); return q_.data(iAx); }
  double* handle_to_qd(const std::string& name) {DEF_iAX(name); return qd_.data(iAx); }
  double* handle_to_qdd(const std::string& name) {DEF_iAX(name); return qdd_.data(iAx); }
  double* handle_to_effort(const std::string& name) {DEF_iAX(name); return effort_.data(iAx); }
  double* handle_to_external_effort(const std::string& name) {DEF_iAX(name); return external_effort_.data(iAx); }
  
  ChainStateN() { };
  ChainStateN(ChainInterfacePtr kin)
  {   
    assert(kin); 
    init(kin);
  } 
  
  ChainStateN(const ChainStateN& cpy)
  {
    copy(cpy); 
  }
  ChainStateN& operator=(const ChainStateN& rhs)
  {
    copy(rhs); 
    return *this;
  }


  virtual bool init(ChainInterfacePtr kin)
  {
    if(!kin)
    {
      return false; 
    }
    kin_ = kin;
    this->q_.resize(this->kin_->nAx());
    this->qd_.resize(this->kin_->nAx());
    this->qdd_.resize(this->kin_->nAx());
    this->effort_.resize(this->kin_->nAx());
    this->external_effort_.resize(this->kin_->nAx());
    this->setZero();
    return true;
  }
  
  
  
  void copy(const ChainStateN<-1>& cpy)
  {
    this->kin_ = cpy.kin_ ;
    this->q_ = cpy.q_;
    this->qd_ = cpy.qd_;
    this->qdd_ = cpy.qdd_;
    this->effort_ = cpy.effort_;
    this->external_effort_ = cpy.external_effort_;
    updateTransformations();
  }
  
  void setZero()
  {
    this->q_.value().setZero();
    this->qd_.value().setZero();
    this->qdd_.value().setZero();
    this->effort_.value().setZero();
    this->external_effort_.value().setZero();
    updateTransformations();
  }
};





}  // rosdyn

#undef TP
#undef SP

#endif  //  ROSDYN_UTILITIES__CHAIN_STATUS_IMPL__H