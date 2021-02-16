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

#include <state_space_filters/filtered_values.h>
#include <rosdyn_core/primitives.h>

#define DEF_iAX(name)\
  int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list");

#define CHECK_iAx(iAx)\
  if(iAx>=eigen_utils::rows(q_.value())) throw std::runtime_error("Index out of range!");

namespace rosdyn
{

/**
 * @class ChainStateN
 *
 * The class does not own the pointer to ChainInterface, but only a raw pointer is used.
 * This allow the usage also in the stack as stati object.
 */
template<int N, int MaxN = N>
class ChainStateN
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<ChainStateN> Ptr;
  typedef std::shared_ptr<ChainStateN const> ConstPtr;

  using Value = typename eigen_control_toolbox::FilteredValue<N,MaxN>::Value;
  using JacobianMatrix = Eigen::Matrix<double,6,N, Eigen::ColMajor,6, MaxN>;

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
  const JacobianMatrix&  jacobian( ) const { return jacobian_; }

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
  ChainStateN& updateTransformations(ChainPtr kin, int ffwd_kin_type);

  template<int n=N, std::enable_if_t<n==1,int> = 0>
  ChainStateN& updateTransformations(ChainPtr kin, int ffwd_kin_type);

  template<int n=N, std::enable_if_t<n!=1,int> = 0>
  ChainStateN& updateTransformations(Chain& kin, int ffwd_kin_type);

  template<int n=N, std::enable_if_t<n==1,int> = 0>
  ChainStateN& updateTransformations(Chain& kin, int ffwd_kin_type);

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

  ChainStateN() = default;
  ChainStateN(ChainPtr chain);
  ChainStateN(Chain&   chain);
  ChainStateN(const ChainStateN& cpy) = delete;
  ChainStateN(ChainStateN&& cpy) = delete;
  ChainStateN& operator=(const ChainStateN& rhs) = delete;
  ChainStateN& operator=(ChainStateN&& rhs) = delete;
  ~ChainStateN() = default;

  virtual bool init(ChainPtr kin);
  virtual bool init(Chain& kin);
  void setZero(ChainPtr kin);
  void setZero(Chain& kin);

  enum CopyType { ONLY_JOINT, ONLY_CART, FULL_STATE };
  void copy(const ChainStateN<N,MaxN>& cpy, CopyType what);

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
  JacobianMatrix jacobian_;

  int index(const std::string& name) const
  {
      auto it = std::find(joint_names_.begin(), joint_names_.end(), name);
      return(it == joint_names_.end()) ? -1 : std::distance(joint_names_.begin(),it);
  }
  std::vector<std::string> joint_names_;
};


/**
 * @brief ChainState
 */
typedef ChainStateN<-1, rosdyn::max_num_axes> ChainState;
typedef ChainState::Ptr                       ChainStatePtr;
typedef ChainState::ConstPtr                  ChainStateConstPtr;

/**
 *
 */
typedef ChainStateN<-1>         ChainStateX;
typedef ChainStateX::Ptr        ChainStateXPtr;
typedef ChainStateX::ConstPtr   ChainStateXConstPtr;


/**
 * Define the ChainStateN# iwth a pre-defined number of axes
 */

#define DEFINE_ChainStateN_PTR_STATIC_DIMENSION(nAx)\
using ChainState ## nAx = ChainStateN<nAx>;\
typedef std::shared_ptr<ChainStateN<nAx>      > ChainState ## nAx ## Ptr;\
typedef std::shared_ptr<ChainStateN<nAx> const> ChainState ## nAx ## ConstPtr;\

DEFINE_ChainStateN_PTR_STATIC_DIMENSION(1)
DEFINE_ChainStateN_PTR_STATIC_DIMENSION(3)
DEFINE_ChainStateN_PTR_STATIC_DIMENSION(6)
DEFINE_ChainStateN_PTR_STATIC_DIMENSION(7)
}

namespace std
{
template<int N, int MaxN = N>
std::string to_string(const rosdyn::ChainStateN<N,MaxN>& chain);
}

#include <rosdyn_utilities/internal/chain_state_impl.h>


#endif  // rosdyn__CNR_HANDLES_UTILS_H
