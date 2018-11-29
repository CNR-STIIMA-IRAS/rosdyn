#ifndef __ITIA_DYNAMICS_CORE__SPRING__
#define __ITIA_DYNAMICS_CORE__SPRING__

# include <rosdyn_core/base_component.h>


namespace rosdyn
{

class IdealSpring: public ComponentBase
{
protected:
  double m_elasticity;
  double m_offset;
  
public:
  IdealSpring(const std::string& joint_name, const std::string& robot_name, const ros::NodeHandle& nh): ComponentBase(joint_name, robot_name, nh)
  {
    m_type="spring";
    loadParametersAndConstants();
    
    m_nominal_parameters.resize(2);
    m_elasticity = m_nominal_parameters(0)=m_parameters_map.at("elasticity");
    m_offset = m_nominal_parameters(1)=m_parameters_map.at("offset_effort");
    
    m_regressor.resize(m_joints_number, 2);
    m_regressor.setZero();
  };
  virtual Eigen::VectorXd getTorque(const Eigen::Ref<Eigen::VectorXd>& q,  const Eigen::Ref<Eigen::VectorXd>& Dq, const Eigen::Ref<Eigen::VectorXd>& DDq)
  {
    m_torques(m_component_joint_number) = m_elasticity*q(m_joints_number)+m_offset;
    return m_torques;
  };
  virtual Eigen::MatrixXd getRegressor(const Eigen::Ref<Eigen::VectorXd>& q,  const Eigen::Ref<Eigen::VectorXd>& Dq, const Eigen::Ref<Eigen::VectorXd>& DDq)
  {
    m_regressor(m_component_joint_number, 0) = q(m_component_joint_number);
    m_regressor(m_component_joint_number, 1) = 1;
    return m_regressor;
  }
  bool setParameters(const Eigen::Ref<Eigen::VectorXd>& parameters)
  {
    if (parameters.rows() != m_nominal_parameters.rows() )
    {
      ROS_WARN("dimensions mismatch between new parameters and the nominal one");
      return false;
    }
    m_nominal_parameters=parameters;
    
    m_parameters_map.at("elasticity")=m_nominal_parameters(0);
    m_parameters_map.at("offset_effort")=m_nominal_parameters(1);
    return true;
  }  
};

}


#endif