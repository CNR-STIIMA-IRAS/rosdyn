#ifndef ROSDYN_CORE__CHAIN_STATUS__H
#define ROSDYN_CORE__CHAIN_STATUS__H

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <rosdyn_core/primitives.h>
#include <rosdyn_core/chain_interface.h>

namespace rosdyn
{

class ChainState
{
private:
  Eigen::VectorXd               q_;
  Eigen::VectorXd               qd_;
  Eigen::VectorXd               qdd_;
  Eigen::VectorXd               effort_;
  
  Eigen::Affine3d               Tbt_;
  Eigen::Matrix<double, 6, 1>   twist_;
  Eigen::Matrix<double, 6, 1>   twistd_;
  Eigen::Matrix6Xd              jacobian_;

  int  index (const std::string& name) const
  {
    auto it = std::find(kin_->jointNames().begin(), kin_->jointNames().end(), name);
    return (it == kin_->jointNames().end()) ? -1 : std::distance(kin_->jointNames().begin(),it);
  }
  
  ChainInterfacePtr kin_;
public:
  
  typedef std::shared_ptr<ChainState> Ptr;
  typedef std::shared_ptr<ChainState const> ConstPtr;

  // GETTER
  const std::string&               jointName  (const size_t iAx) const { return kin_->jointNames().at(iAx);}
  const std::vector<std::string>&  jointNames () const { return kin_->jointNames();}
  const size_t                     nAx        () const { return kin_->nAx(); }
  const Eigen::VectorXd&           q          () const { return q_;          }
  const Eigen::VectorXd&           qd         () const { return qd_;         }
  const Eigen::VectorXd&           qdd        () const { return qdd_;        }
  const Eigen::VectorXd&           effort     () const { return effort_;     }

  const double& q      (const size_t& iAx) const { if(iAx>=nAx()) throw std::runtime_error("Index out of range!");return q_     (iAx); }
  const double& qd     (const size_t& iAx) const { if(iAx>=nAx()) throw std::runtime_error("Index out of range!");return qd_    (iAx); }
  const double& qdd    (const size_t& iAx) const { if(iAx>=nAx()) throw std::runtime_error("Index out of range!");return qdd_   (iAx); }
  const double& effort (const size_t& iAx) const { if(iAx>=nAx()) throw std::runtime_error("Index out of range!");return effort_(iAx); }

  const double& q     (const std::string& name) const {int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list"); return q     (iAx); }
  const double& qd    (const std::string& name) const {int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list"); return qd    (iAx); }
  const double& qdd   (const std::string& name) const {int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list"); return qdd   (iAx); }
  const double& effort(const std::string& name) const {int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list"); return effort(iAx); }

  // SETTER
  Eigen::VectorXd&  q          () { return q_;          }
  Eigen::VectorXd&  qd         () { return qd_;         }
  Eigen::VectorXd&  qdd        () { return qdd_;        }
  Eigen::VectorXd&  effort     () { return effort_;     }

  double& q      (const size_t& iAx) { return q_     (iAx); }
  double& qd     (const size_t& iAx) { return qd_    (iAx); }
  double& qdd    (const size_t& iAx) { return qdd_   (iAx); }
  double& effort (const size_t& iAx) { return effort_(iAx); }

  double& q     (const std::string& name) {int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list"); return q     (iAx); }
  double& qd    (const std::string& name) {int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list"); return qd    (iAx); }
  double& qdd   (const std::string& name) {int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list"); return qdd   (iAx); }
  double& effort(const std::string& name) {int iAx = index(name); if(iAx==-1) throw std::runtime_error("Name not in the joint_names list"); return effort(iAx); }
  
  ChainInterfacePtr getKin() const { return kin_; }
  ChainState& updateTransformation( );
  
  const Eigen::Affine3d&             toolPose  ( ) const { return Tbt_;       }
  const Eigen::Matrix<double, 6, 1>& twist     ( ) const { return twist_;     }
  const Eigen::Matrix<double, 6, 1>& twistd    ( ) const { return twistd_;    }
  const Eigen::Matrix6Xd             jacobian  ( ) const { return jacobian_;  }

  ChainState() = delete;
  ChainState(ChainInterfacePtr kin);
  ChainState(const ChainState& cpy);
  ChainState& operator=(const ChainState& rhs);

  void setZero();
};

typedef ChainState::Ptr ChainStatePtr;
typedef ChainState::ConstPtr ChainStateConstPtr;

}

#include <rosdyn_core/internal/chain_state_impl.h>

#endif  // rosdyn__CNR_HANDLES_UTILS_H



