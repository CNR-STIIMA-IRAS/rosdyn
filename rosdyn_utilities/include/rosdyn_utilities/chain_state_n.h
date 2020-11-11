#ifndef ROSDYN_UTILITIES__CHAIN_STATE_N__H
#define ROSDYN_UTILITIES__CHAIN_STATE_N__H

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <rosdyn_core/primitives.h>
#include <rosdyn_utilities/chain_interface.h>
#include <rosdyn_utilities/filtered_values.h>

#define DEF_iAX(name)\
  int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list");

#define CHECK_iAx(iAx)\
  if(iAx>=nAx()) throw std::runtime_error("Index out of range!");

namespace rosdyn
{

/**
 * @class ChainStateN 
 */
template<int N, int MaxN = N>
class ChainStateN
{
protected:
  rosdyn::FilteredValue<N,MaxN> q_;
  rosdyn::FilteredValue<N,MaxN> qd_;
  rosdyn::FilteredValue<N,MaxN> qdd_;
  rosdyn::FilteredValue<N,MaxN> effort_;
  rosdyn::FilteredValue<N,MaxN> external_effort_;
  
  Eigen::Affine3d Tbt_;
  Eigen::Vector6d twist_;
  Eigen::Vector6d twistd_;
  rosdyn::FilteredValue<6> wrench_;
  Eigen::Matrix<double,6,N> jacobian_;

  int index(const std::string& name) const
  {
    auto it = std::find(kin_->jointNames().begin(), kin_->jointNames().end(), name);
    return(it == kin_->jointNames().end()) ? -1 : std::distance(kin_->jointNames().begin(),it);
  }

  rosdyn::ChainInterfacePtr kin_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<ChainStateN> Ptr;
  typedef std::shared_ptr<ChainStateN const> ConstPtr;

  using Value = typename rosdyn::FilteredValue<N,MaxN>::Value;

  // GETTER
  const std::string& jointName(const size_t iAx) const { return kin_->jointNames().at(iAx);}
  const std::vector<std::string>& jointNames() const { return kin_->jointNames();}
  const size_t nAx() const { return kin_->nAx(); }
  const Value& q() const { return q_.value();  }
  const Value& qd() const { return qd_.value(); }
  const Value& qdd() const { return qdd_.value();}
  const Value& effort() const { return effort_.value(); }
  const Value& external_effort() const { return external_effort_.value(); }

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
  const Eigen::Matrix<double,6,N>& jacobian( ) const { return jacobian_;  }
  
  // SETTER
  Value& q() { return q_.value();  }
  Value& qd() { return qd_.value(); }
  Value& qdd() { return qdd_.value();}
  Value& effort() { return effort_.value(); }
  Value& external_effort() { return external_effort_.value(); }
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

  rosdyn::FilteredValue<N,MaxN>& qFilteredValue() {return q_;}
  rosdyn::FilteredValue<N,MaxN>& qdFilteredValue() {return qd_;}
  rosdyn::FilteredValue<N,MaxN>& qddFilteredValue() {return qdd_;}
  rosdyn::FilteredValue<N,MaxN>& effortFilteredValue() {return effort_;}
  rosdyn::FilteredValue<N,MaxN>& externalEffortFilteredValue() {return external_effort_;}
  rosdyn::FilteredValue<6>&       wrenchFilteredValue() {return wrench_;}

  // METHODS
  ChainInterfacePtr getKin() const { return kin_; }
  ChainStateN& updateTransformations( bool effort_to_wrench = true );
  ChainStateN& updateTransformations( const Value& q,
                                      const Value& qd,
                                      const Value& qdd,
                                      const Value& external_effort);
  
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
  ChainStateN(ChainInterfacePtr kin);
  ChainStateN(const ChainStateN& cpy);
  ChainStateN& operator=(const ChainStateN& rhs);

  virtual bool init(ChainInterfacePtr kin);
  void copy(const ChainStateN<N,MaxN>& cpy, bool update_transform = true);
  void setZero();
};

}

/**
 * impl and specialization
 */
#include <rosdyn_utilities/internal/chain_state_n_impl.h>



#endif  // ROSDYN_UTILITIES__CHAIN_STATE_N__H



