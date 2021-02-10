#ifndef ROSDYN_UTILITIES__CHAIN_STATUS__H
#define ROSDYN_UTILITIES__CHAIN_STATUS__H

#include <thread>
#include <type_traits>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <eigen_state_space_systems/filters/filtered_values.h>
#include <rosdyn_core/primitives.h>

#define DEF_iAX(name)\
  int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list");

#define CHECK_iAx(iAx)\
  if(iAx>=eigen_utils::rows(q_.value())) throw std::runtime_error("Index out of range!");

namespace rosdyn
{

/**
 * @class ChainState
 *
 * The class does not own the pointer to ChainInterface, but only a raw pointer is used.
 * This allow the usage also in the stack as stati object.
 */
template<int N, int MaxN = N>
class ChainState
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<ChainState> Ptr;
  typedef std::shared_ptr<ChainState const> ConstPtr;

  using Value = typename eigen_control_toolbox::FilteredValue<N,MaxN>::Value;

  // GETTER
  const Value&  q() const { return q_.value();  }
  const Value&  qd() const { return qd_.value(); }
  const Value&  qdd() const { return qdd_.value();}
  const Value&  effort() const { return effort_.value(); }
  const Value&  external_effort() const { return external_effort_.value(); }

  double q(const int& iAx) const {CHECK_iAx(iAx); return q_.value(iAx); }
  double qd(const int& iAx) const {CHECK_iAx(iAx); return qd_.value(iAx); }
  double qdd(const int& iAx) const {CHECK_iAx(iAx); return qdd_.value(iAx); }
  double effort(const int& iAx) const {CHECK_iAx(iAx); return effort_.value(iAx); }
  double external_effort(const int& iAx) const {CHECK_iAx(iAx); return external_effort_.value(iAx); }

  double q(const std::string& name) const {DEF_iAX(name); return q(iAx); }
  double qd(const std::string& name) const {DEF_iAX(name); return qd(iAx); }
  double qdd(const std::string& name) const {DEF_iAX(name); return qdd(iAx); }
  double effort(const std::string& name) const {DEF_iAX(name); return effort(iAx); }
  double external_effort(const std::string& name) const {DEF_iAX(name); return external_effort(iAx); }

  const Eigen::Affine3d& toolPose( ) const { return Tbt_;   }
  const Eigen::Vector6d& twist( ) const { return twist_; }
  const Eigen::Vector6d& twistd( ) const { return twistd_;}
  const Eigen::Vector6d& wrench( ) const { return wrench_.value();}
  const Eigen::Matrix<double,6,N, Eigen::ColMajor,6, MaxN>& jacobian( ) const { return jacobian_; }

  // SETTER
  Value& q() { return q_.value();  }
  Value& qd() { return qd_.value(); }
  Value& qdd() { return qdd_.value();}
  Value& effort() { return effort_.value(); }
  Value& external_effort() { return external_effort_.value(); }
  Eigen::Vector6d& wrench( ) { return wrench_.value();}

  double& q(const int& iAx) { return q_.value(iAx); }
  double& qd(const int& iAx) { return qd_.value(iAx); }
  double& qdd(const int& iAx) { return qdd_.value(iAx); }
  double& effort(const int& iAx) { return effort_.value(iAx); }
  double& external_effort(const int& iAx) { return external_effort_.value(iAx); }

  double& q(const std::string& name) {DEF_iAX(name); return q(iAx); }
  double& qd(const std::string& name) {DEF_iAX(name); return qd(iAx); }
  double& qdd(const std::string& name) {DEF_iAX(name); return qdd(iAx); }
  double& effort(const std::string& name) {DEF_iAX(name); return effort(iAx); }
  double& external_effort(const std::string& name) {DEF_iAX(name); return external_effort(iAx); }

  eigen_control_toolbox::FilteredValue<N,MaxN>& qFilteredValue() {return q_;}
  eigen_control_toolbox::FilteredValue<N,MaxN>& qdFilteredValue() {return qd_;}
  eigen_control_toolbox::FilteredValue<N,MaxN>& qddFilteredValue() {return qdd_;}
  eigen_control_toolbox::FilteredValue<N,MaxN>& effortFilteredValue() {return effort_;}
  eigen_control_toolbox::FilteredValue<N,MaxN>& externalEffortFilteredValue() {return external_effort_;}
  eigen_control_toolbox::FilteredValue<6>&      wrenchFilteredValue() {return wrench_;}

  // METHODS
  enum KinematicsType
  {
    ZERO_ORDER   = 0b00000001, // ONLY POSITION FFWD
    FIRST_ORDER  = 0b00000011, // POS AND VEL FFWD
    SECOND_ORDER = 0b00000111, // POS, VEL, ACC FFWD
    FFWD_STATIC  = 0b00001000, // from EFFORT to wrench
    INV_STATIC   = 0b00010000  // from wrench to effort
  };

  template<int n=N, std::enable_if_t<n!=1,int> = 0>
  ChainState& updateTransformations(ChainPtr kin, int ffwd_kin_type);

  template<int n=N, std::enable_if_t<n==1,int> = 0>
  ChainState& updateTransformations(ChainPtr kin, int ffwd_kin_type);

  template<int n=N, std::enable_if_t<n!=1,int> = 0>
  ChainState& updateTransformations(Chain& kin, int ffwd_kin_type);

  template<int n=N, std::enable_if_t<n==1,int> = 0>
  ChainState& updateTransformations(Chain& kin, int ffwd_kin_type);

  double* handle_to_q(const int& iAx=0) {CHECK_iAx(iAx); return q_.data(iAx); }
  double* handle_to_qd(const int& iAx=0) {CHECK_iAx(iAx); return qd_.data(iAx); }
  double* handle_to_qdd(const int& iAx=0) {CHECK_iAx(iAx); return qdd_.data(iAx); }
  double* handle_to_effort(const int& iAx=0) {CHECK_iAx(iAx); return effort_.data(iAx); }
  double* handle_to_external_effort(const int& iAx=0) {CHECK_iAx(iAx); return external_effort_.data(iAx); }

  double* handle_to_q(const std::string& name) {DEF_iAX(name); return q_.data(iAx); }
  double* handle_to_qd(const std::string& name) {DEF_iAX(name); return qd_.data(iAx); }
  double* handle_to_qdd(const std::string& name) {DEF_iAX(name); return qdd_.data(iAx); }
  double* handle_to_effort(const std::string& name) {DEF_iAX(name); return effort_.data(iAx); }
  double* handle_to_external_effort(const std::string& name) {DEF_iAX(name); return external_effort_.data(iAx); }

  ChainState() = default;
  ChainState(ChainPtr chain);
  ChainState(Chain&   chain);
  ChainState(const ChainState& cpy) = delete;
  ChainState(ChainState&& cpy) = delete;
  ChainState& operator=(const ChainState& rhs) = delete;
  ChainState& operator=(ChainState&& rhs) = delete;
  ~ChainState() = default;

  virtual bool init(ChainPtr kin);
  virtual bool init(Chain& kin);
  void setZero(ChainPtr kin);
  void setZero(Chain& kin);

  enum CopyType { ONLY_JOINT, ONLY_CART, FULL_STATE };
  void copy(const ChainState<N,MaxN>& cpy, CopyType what);

  const std::vector<std::string> getJointNames() const { return joint_names_; }
   int nAx() const { return int(joint_names_.size()); }

protected:
  eigen_control_toolbox::FilteredValue<N,MaxN> q_;
  eigen_control_toolbox::FilteredValue<N,MaxN> qd_;
  eigen_control_toolbox::FilteredValue<N,MaxN> qdd_;
  eigen_control_toolbox::FilteredValue<N,MaxN> effort_;
  eigen_control_toolbox::FilteredValue<N,MaxN> external_effort_;

  Eigen::Affine3d Tbt_;
  Eigen::Vector6d twist_;
  Eigen::Vector6d twistd_;
  eigen_control_toolbox::FilteredValue<6> wrench_;
  Eigen::Matrix<double,6,N, Eigen::ColMajor,6, MaxN> jacobian_;

  int index(const std::string& name) const
  {
      auto it = std::find(joint_names_.begin(), joint_names_.end(), name);
      return(it == joint_names_.end()) ? -1 : std::distance(joint_names_.begin(),it);
  }
  std::vector<std::string> joint_names_;
};

using ChainStateX = ChainState<-1>;
typedef ChainStateX::Ptr  ChainStateXPtr;
typedef ChainStateX::ConstPtr  ChainStateXConstPtr;


/**
 * Define the ChainState# iwth a pre-defined number of axes
 */ 

#define DEFINE_CHAINSTATE_PTR_STATIC_DIMENSION(nAx)\
using ChainState ## nAx = ChainState<nAx>;\
typedef std::shared_ptr<ChainState<nAx>      > ChainState ## nAx ## Ptr;\
typedef std::shared_ptr<ChainState<nAx> const> ChainState ## nAx ## ConstPtr;\

DEFINE_CHAINSTATE_PTR_STATIC_DIMENSION(1)
DEFINE_CHAINSTATE_PTR_STATIC_DIMENSION(3)
DEFINE_CHAINSTATE_PTR_STATIC_DIMENSION(6)
DEFINE_CHAINSTATE_PTR_STATIC_DIMENSION(7)
}

#include <rosdyn_utilities/internal/chain_state_impl.h>


#endif  // rosdyn__CNR_HANDLES_UTILS_H
