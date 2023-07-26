#ifndef ROSDYN_CORE_INTERNAL_PRIMITIVES_IMPL_H
#define ROSDYN_CORE_INTERNAL_PRIMITIVES_IMPL_H

#include <chrono>
#include <rosdyn_core/primitives.h>

namespace rosdyn
{
    


inline int enforceLimitsFromRobotDescriptionParam(double& Dq_max, 
                                                    double& DDq_max, 
                                                      const std::string& name,
                                                        const std::string& full_param_path, 
                                                          std::string& what)
{
  //=============================================================
  if(!ros::param::has(full_param_path))
  {
    what = "Parameter '" + full_param_path + "' is not in rosparam server.\n";
    return -1;
  }

  std::string joint_limits_param = full_param_path +"/joint_limits/" + name;
  if(!ros::param::has(joint_limits_param))
  {
    what = "Parameter '" + joint_limits_param + "' is not in rosparam server.\n";
    return -1;
  }
  //=============================================================

  try
  {
    bool has_velocity_limits;
    if(!ros::param::get(joint_limits_param + "/has_velocity_limits", has_velocity_limits))
    {
      has_velocity_limits = false;
    }
    bool has_acceleration_limits;
    if (!ros::param::get(joint_limits_param +  "/has_acceleration_limits", has_acceleration_limits))
    {
      has_acceleration_limits = false;
    }
    if (has_velocity_limits)
    {
      double vel = NAN;
      if(!ros::param::get(joint_limits_param + "/max_velocity", vel))
      {
        what += (what.length()>0 ? "\n" : "")
             + joint_limits_param + "/max_velocity is not defined. URDF value is superimposed"
            + " (vel max=" + std::to_string(Dq_max)+ ")";
      }
      Dq_max = std::isnan(vel) || (vel<=0) ? Dq_max : vel;

    }

    if (has_acceleration_limits)
    {
      double acc = NAN;
      if (!ros::param::get(joint_limits_param +  "/max_acceleration", acc))
      {
        what += (what.length()>0 ? "\n" : "")
             + joint_limits_param + "/max_acceleration is not defined. The superimposed value is ten times the max vel"
             + "(acc max = " + std::to_string(10 * Dq_max)+ ")";
      }
      DDq_max = std::isnan(acc) || (acc<=0) ?  10 * Dq_max : acc;
    }
  }
  catch (...)
  {
    what +="Unknown excpetion in getting data from joint ''" + name + "'.";
    return -1;
  }
  return what.length()>0 ? 0 : 1;
}

inline Chain::Chain(const Chain& cpy) : rdyn::Chain(cpy)
{

  rosdyn::LinkPtr root_link = cpy.getLinks().front();
  std::string base_link_name = cpy.getLinksName().front();
  std::string ee_link_name = cpy.getLinksName().back();
  Eigen::Vector3d gravity = cpy.getGravity();
  std::string error;
  if(!this->init(error, root_link, base_link_name,ee_link_name,gravity))
  {
    throw std::runtime_error(error.c_str());
  }
}


inline Chain::Chain(const rosdyn::LinkPtr& root_link, const std::string& base_link_name, const std::string& ee_link_name, const Eigen::Vector3d& gravity)
  : Chain()
{
  std::string error;
  if(!this->init(error, root_link, base_link_name,ee_link_name,gravity))
  {
    throw std::runtime_error(error.c_str());
  }
}

inline Chain::Chain(const urdf::Model& model,
        const std::string& base_link_name, const std::string& ee_link_name, const Eigen::Vector3d& gravity)
{
  rosdyn::LinkPtr root_link(new rosdyn::Link());
  root_link->fromUrdf(model.root_link_);
  std::string error;
  if(!this->init(error, root_link, base_link_name,ee_link_name,gravity))
  {
    throw std::runtime_error(error.c_str());
  }
}

inline Chain::Chain(const std::string& robot_description, 
            const std::string& base_link_name, const std::string& ee_link_name, const Eigen::Vector3d& gravity)
{
  urdf::Model model;
  model.initParam(robot_description);
  rosdyn::LinkPtr root_link(new rosdyn::Link());
  root_link->fromUrdf(model.root_link_);
  std::string error;
  if(!this->init(error, root_link, base_link_name,ee_link_name,gravity))
  {
    throw std::runtime_error(error.c_str());
  }
}

inline Chain& Chain::operator=(const Chain& rhs)
{
  m_links.clear();
  m_joints.clear();
  m_links_name.clear();
  m_joints_name.clear();
  m_moveable_joints_name.clear();
  m_active_joints_name.clear();
  m_parent_moveable_joints_of_link.clear();

  rosdyn::LinkPtr root_link = rhs.getLinks().front();
  std::string base_link_name = rhs.getLinksName().front();
  std::string ee_link_name = rhs.getLinksName().back();
  Eigen::Vector3d gravity = rhs.getGravity();
  std::string error;
  if(!this->init(error, root_link, base_link_name,ee_link_name,gravity))
  {
    throw std::runtime_error(error.c_str());
  }
  return *this;
}

inline int Chain::enforceLimitsFromRobotDescriptionParam(const std::string& full_param_path, std::string& error)
{
  for(auto const & name : m_moveable_joints_name)
  {
    auto & joint = m_joints.at( m_joints_name.at(name) );
    std::string what;
    int res = rosdyn::enforceLimitsFromRobotDescriptionParam(joint->getDQMax(), joint->getDDQMax(), name, full_param_path, what);
    if(res==-1)
    {
      error += what;
      return -1;
    }
    else if(res==0)
    {
      error += (error.length()>0? "\n":"") + what;
    }
  }

  for (unsigned int idx = 0; idx < m_active_joints_number; idx++)
  {
    auto& jnt = m_joints.at(m_active_joints.at(idx));
    m_q_max(idx) = jnt->getQMax();
    m_q_min(idx) = jnt->getQMin();
    m_Dq_max(idx) = jnt->getDQMax();
    m_DDq_max(idx) = jnt->getDDQMax();
    m_tau_max(idx) = jnt->getTauMax();
  }
  return error.length()>0 ? 0 : 1;
}

inline bool Chain::setInputJointsName(const std::vector<std::string>& joints_name)
{
  std::string what; 
  bool ret = this->setInputJointsName(joints_name, what);
  if(what.length())
  {
    ROS_ERROR("%s",what.c_str());
  }
  
  return ret;
}


inline rosdyn::ChainPtr createChain(const urdf::ModelInterface& urdf_model_interface,
    const std::string& base_frame, const std::string& tool_frame, const Eigen::Vector3d& gravity)
{
  rosdyn::LinkPtr root_link(new rosdyn::Link());
  root_link->fromUrdf(urdf_model_interface.root_link_);
  rosdyn::ChainPtr chain(new rosdyn::Chain(root_link, base_frame, tool_frame, gravity));
  if (!chain->isOk())
    chain.reset();
  return chain;
}

inline rosdyn::ChainPtr createChain(const rosdyn::ChainPtr& cpy)
{
  rosdyn::LinkPtr root_link = cpy->getLinks().front();
  std::string base_link_name = cpy->getLinksName().front();
  std::string ee_link_name = cpy->getLinksName().back();
  Eigen::Vector3d gravity = cpy->getGravity();
  rosdyn::ChainPtr chain(new rosdyn::Chain(root_link, base_link_name, ee_link_name, gravity));
  return chain;
}

inline rosdyn::ChainPtr createChain(const rosdyn::Chain& cpy)
{
  rosdyn::LinkPtr root_link = cpy.getLinks().front();
  std::string base_link_name = cpy.getLinksName().front();
  std::string ee_link_name = cpy.getLinksName().back();
  Eigen::Vector3d gravity = cpy.getGravity();
  rosdyn::ChainPtr chain(new rosdyn::Chain(root_link, base_link_name, ee_link_name, gravity));
  return chain;
}


}  // namespace rosdyn

#endif  // ROSDYN_CORE_INTERNAL_PRIMITIVES_IMPL_H
