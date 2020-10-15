#ifndef ROSDYN_CORE__CHAIN_INTERFACE__H
#define ROSDYN_CORE__CHAIN_INTERFACE__H

#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <rosdyn_core/primitives.h>

namespace rosdyn
{

class ChainInterface
{

private:
  urdf::ModelInterfaceSharedPtr m_model;
  std::string                   m_base_link;
  std::string                   m_tool_link;
  rosdyn::ChainPtr              m_chain;

  std::vector<std::string>      m_joint_names;
  std::vector<std::string>      m_link_names;
  size_t                        m_nAx;
  Eigen::VectorXd               m_upper_limit;
  Eigen::VectorXd               m_lower_limit;
  Eigen::VectorXd               m_qd_limit;
  Eigen::VectorXd               m_qdd_limit;
  Eigen::VectorXd               m_effort_limit;

  Eigen::IOFormat               m_cfrmt;

public:
  const size_t& nAx                         ( ) const { return m_nAx;         }
  const Eigen::VectorXd& upperLimit         ( ) const { return m_upper_limit; }
  const Eigen::VectorXd& lowerLimit         ( ) const { return m_lower_limit; }
  const Eigen::VectorXd& speedLimit         ( ) const { return m_qd_limit;    }
  const Eigen::VectorXd& accelerationLimit  ( ) const { return m_qdd_limit;   }
  const Eigen::VectorXd& effortLimit        ( ) const { return m_effort_limit;}
  const std::vector<std::string>& jointNames( ) const { return m_joint_names; }
  const double& upperLimit         (size_t iAx) const { return m_upper_limit(iAx); }
  const double& lowerLimit         (size_t iAx) const { return m_lower_limit(iAx); }
  const double& speedLimit         (size_t iAx) const { return m_qd_limit(iAx);    }
  const double& accelerationLimit  (size_t iAx) const { return m_qdd_limit(iAx);   }
  const double& eeffortLimit       (size_t iAx) const { return m_effort_limit(iAx);   }
  const std::string& jointName     (size_t iAx) const { return m_joint_names.at(iAx); }
  double upperLimit         (const std::string& name) const {int idx=jointIndex(name); return idx==-1 ? NAN : m_upper_limit (idx); }
  double lowerLimit         (const std::string& name) const {int idx=jointIndex(name); return idx==-1 ? NAN : m_lower_limit (idx); }
  double speedLimit         (const std::string& name) const {int idx=jointIndex(name); return idx==-1 ? NAN : m_qd_limit    (idx); }
  double accelerationLimit  (const std::string& name) const {int idx=jointIndex(name); return idx==-1 ? NAN : m_qdd_limit   (idx); }
  double eeffortLimit       (const std::string& name) const {int idx=jointIndex(name); return idx==-1 ? NAN : m_effort_limit(idx); }
  int jointIndex (const std::string& name) const
  {
    auto it = std::find(m_joint_names.begin(),m_joint_names.end(), name);
    return it == m_joint_names.end() ? -1 : std::distance(m_joint_names.begin(), it);
  }

  const std::string&          linkNames (size_t i) const { return m_link_names.at(i);}
  const std::vector<std::string>&    linkNames ( ) const { return m_link_names;}
  const std::string&                 baseLink  ( ) const { return m_base_link; }
  const std::string&                 baseFrame ( ) const { return baseLink();  }
  const std::string&                 toolLink  ( ) const { return m_tool_link; }
  const std::string&                 toolFrame ( ) const { return toolLink();  }
  
  rosdyn::ChainPtr       getChain  ( )                { return m_chain;     }
  rosdyn::ChainPtr       getChain  ( const std::string& from, const std::string& to);

  bool init(ros::NodeHandle& parent_nh, ros::NodeHandle& nh, std::stringstream& report);


  bool saturateSpeed(Eigen::Ref<Eigen::VectorXd> qd_target,
                     const Eigen::Ref<const Eigen::VectorXd> qd_actual,
                     const Eigen::Ref<const Eigen::VectorXd> q_actual,
                     double dt,
                     double max_velocity_multiplier,
                     bool preserve_direction,
                     std::stringstream* report);

  bool saturateSpeed(Eigen::Ref<Eigen::VectorXd> qd_target,
                     const Eigen::Ref<const Eigen::VectorXd> qd_actual,
                     double dt,
                     double max_velocity_multiplier,
                     bool preserve_direction,
                     std::stringstream* report);

  bool saturateSpeed(Eigen::Ref<Eigen::VectorXd> qd_target,
                     double max_velocity_multiplier,
                     bool preserve_direction,
                     std::stringstream* report);

  bool saturatePosition(Eigen::Ref<Eigen::VectorXd> q_target, std::stringstream* report);

};

typedef std::shared_ptr<ChainInterface> ChainInterfacePtr;
typedef const std::shared_ptr<ChainInterface const> ChainInterfaceConstPtr;

}  // namespace rosdyn

#include <rosdyn_core/internal/chain_interface_impl.h>

#endif  // ROSDYN_CORE__CHAIN_INTERFACE__H



