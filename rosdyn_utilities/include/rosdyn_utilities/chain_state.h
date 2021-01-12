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

#include <eigen_state_space_systems/filtered_values.h>
#include <rosdyn_utilities/chain_interface.h>

#define DEF_iAX(name)\
  int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list");

#define CHECK_iAx(iAx)\
  if(iAx>=nAx()) throw std::runtime_error("Index out of range!");

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
  const std::string& jointName(const size_t iAx) const { return kin_->jointNames().at(iAx);}
  const std::vector<std::string>& jointNames() const { return kin_->jointNames();}
  const size_t& nAx() const { return kin_->nAx(); }
  const Value&  q() const { return q_.value();  }
  const Value&  qd() const { return qd_.value(); }
  const Value&  qdd() const { return qdd_.value();}
  const Value&  effort() const { return effort_.value(); }
  const Value&  external_effort() const { return external_effort_.value(); }

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

  eigen_control_toolbox::FilteredValue<N,MaxN>& qFilteredValue() {return q_;}
  eigen_control_toolbox::FilteredValue<N,MaxN>& qdFilteredValue() {return qd_;}
  eigen_control_toolbox::FilteredValue<N,MaxN>& qddFilteredValue() {return qdd_;}
  eigen_control_toolbox::FilteredValue<N,MaxN>& effortFilteredValue() {return effort_;}
  eigen_control_toolbox::FilteredValue<N,MaxN>& externalEffortFilteredValue() {return external_effort_;}
  eigen_control_toolbox::FilteredValue<6>&       wrenchFilteredValue() {return wrench_;}

  // METHODS
  enum KinematicsType {
    ZERO_ORDER   = 0b00000001, // ONLY POSITION FFWD
    FIRST_ORDER  = 0b00000011, // POS AND VEL FFWD
    SECOND_ORDER = 0b00000111, // POS, VEL, ACC FFWD
    FFWD_STATIC  = 0b00001000, // from EFFORT to wrench
    INV_STATIC   = 0b00010000  // from wrench to effort
  };
  ChainState& updateTransformations(int ffwd_kin_type);

  void startUpdateTransformationsThread(int ffwd_kin_type, double hz = 10.0);
  void stopUpdateTransformationsThread();

  double* handle_to_q(const size_t& iAx=0) {CHECK_iAx(iAx); return q_.data(iAx); }
  double* handle_to_qd(const size_t& iAx=0) {CHECK_iAx(iAx); return qd_.data(iAx); }
  double* handle_to_qdd(const size_t& iAx=0) {CHECK_iAx(iAx); return qdd_.data(iAx); }
  double* handle_to_effort(const size_t& iAx=0) {CHECK_iAx(iAx); return effort_.data(iAx); }
  double* handle_to_external_effort(const size_t& iAx=0) {CHECK_iAx(iAx); return external_effort_.data(iAx); }

  double* handle_to_q(const std::string& name) {DEF_iAX(name); return q_.data(iAx); }
  double* handle_to_qd(const std::string& name) {DEF_iAX(name); return qd_.data(iAx); }
  double* handle_to_qdd(const std::string& name) {DEF_iAX(name); return qdd_.data(iAx); }
  double* handle_to_effort(const std::string& name) {DEF_iAX(name); return effort_.data(iAx); }
  double* handle_to_external_effort(const std::string& name) {DEF_iAX(name); return external_effort_.data(iAx); }

  ChainState() = default;
  ChainState(ChainInterfacePtr kin);
  ChainState(const ChainState& cpy);
  ChainState& operator=(const ChainState& rhs);
  ~ChainState();

  ChainInterfacePtr getChainInterface() {return kin_;}

  bool initialized() const { return kin_ != nullptr; }
  virtual bool init(ChainInterfacePtr kin);
  void setZero();


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
  Eigen::Matrix<double,6,N> jacobian_;

  int index(const std::string& name) const
  {
    auto it = std::find(kin_->jointNames().begin(), kin_->jointNames().end(), name);
    return(it == kin_->jointNames().end()) ? -1 : std::distance(kin_->jointNames().begin(),it);
  }

  rosdyn::ChainInterfacePtr kin_;

  void updateTransformationsThread();
  double          hz_;
  int             ffwd_kin_type_;
  std::thread     update_transformations_;
  bool            stop_update_transformations_;
  std::mutex      mtx_;

  template<int n=N, std::enable_if_t<n==1, int> =0 >
  ChainState& updateTransformations(const Value* q,
                                    const Value* qd,
                                    const Value* qdd,
                                    const Value* external_effort = nullptr,
                                    const Eigen::Matrix<double,6,1>* wrench = nullptr);

  template<int n=N, std::enable_if_t<n!=1, int> =0 >
  ChainState& updateTransformations(const Value* q,
                                    const Value* qd,
                                    const Value* qdd,
                                    const Value* external_effort = nullptr,
                                    const Eigen::Matrix<double,6,1>* wrench = nullptr);

  void copy(const ChainState<N,MaxN>& cpy, bool update_transform = false);
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
