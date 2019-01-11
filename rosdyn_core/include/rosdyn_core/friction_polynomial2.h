#ifndef __ITIA_DYNAMICS_CORE__FRICTION_POLY2__
#define __ITIA_DYNAMICS_CORE__FRICTION_POLY2__

# include <rosdyn_core/base_component.h>


namespace rosdyn
{
class SecondOrderPolynomialFriction: public ComponentBase
{
protected:
  double m_Dq_threshold;
  double m_Dq_max;
  
  void computeRegressor(const Eigen::Ref<Eigen::VectorXd>& Dq )
  {
    double omega = std::min(std::max(Dq(m_component_joint_number), -m_Dq_max), m_Dq_max);
    double sign_Dq = 0;
    if (omega == 0)
      sign_Dq = 0;
    else if (omega > m_Dq_threshold)
      sign_Dq = 1.0;
    else if (omega < -m_Dq_threshold)
      sign_Dq = -1.0;
    else
      sign_Dq = omega/m_Dq_threshold;
    
    m_regressor(m_component_joint_number, 0) = sign_Dq;
    m_regressor(m_component_joint_number, 1) = omega;
    m_regressor(m_component_joint_number, 2) = pow(omega, 2.0) *sign_Dq;
    
  }
  
public:
  SecondOrderPolynomialFriction(const std::string& joint_name, const std::string& robot_name, const ros::NodeHandle& nh): ComponentBase(joint_name, robot_name, nh)
  {
    
    m_type="friction";
    loadParametersAndConstants();
    
    if (m_constants_map.size()<2)
      throw std::invalid_argument(robot_name+"/" + joint_name + "/friction/constants has wrong dimensions, " + std::to_string(m_constants_map.size()));
    
    if (m_parameters_map.size()<3)
      throw std::invalid_argument(robot_name+"/" + joint_name + "/friction/coefficients has wrong dimensions, " + std::to_string(m_parameters_map.size()));
    
    m_nominal_parameters.resize(3);
    m_nominal_parameters(0)=m_parameters_map.at("coloumb");
    m_nominal_parameters(1)=m_parameters_map.at("first_order_viscous");
    m_nominal_parameters(2)=m_parameters_map.at("second_order_viscous");
    
    m_Dq_threshold = m_constants_map.at("min_velocity");
    if (m_Dq_threshold  <1e-6)
    {
      ROS_WARN("friction/min_velocity must be greater than 1.0e-6, set default value = 1.0e-6");
      m_Dq_threshold = 1.0e-6;
      m_nh.setParam(robot_name+"/"+joint_name+"/friction/constants/min_velocity",m_Dq_threshold);
    }
    m_Dq_max=m_constants_map.at("max_velocity");
    if (m_Dq_max<0)
    {
      ROS_WARN("friction/max_velocity must be positive and smaller than 1.0e6, set default value = 1.0e6");
      m_Dq_threshold = 1.0e6;
      m_nh.setParam(robot_name+"/"+joint_name+"/friction/constants/max_velocity",m_Dq_max);
    }
    m_regressor.resize(m_joints_number, 3);
    m_regressor.setZero();
    
    
    
  };
    
  virtual Eigen::VectorXd getTorque(const Eigen::Ref<Eigen::VectorXd>& q,  const Eigen::Ref<Eigen::VectorXd>& Dq, const Eigen::Ref<Eigen::VectorXd>& DDq)
  {
    computeRegressor(Dq);
    m_torques(m_component_joint_number) = m_regressor.row(m_component_joint_number)*m_nominal_parameters;
    return m_torques;
  };
  
  virtual Eigen::VectorXd getAdditiveTorque(const Eigen::Ref< Eigen::VectorXd >& q, const Eigen::Ref< Eigen::VectorXd >& Dq, const Eigen::Ref< Eigen::VectorXd >& DDq)
  {
    computeRegressor(Dq);
    m_torques(m_component_joint_number) = m_regressor.rightCols(2).row(m_component_joint_number)*m_nominal_parameters.tail(2);
    return m_torques;
  };
  
  virtual Eigen::VectorXd getNonAdditiveTorque(const Eigen::Ref< Eigen::VectorXd >& q, const Eigen::Ref< Eigen::VectorXd >& Dq, const Eigen::Ref< Eigen::VectorXd >& DDq, const Eigen::Ref< Eigen::VectorXd >& additive_torque)
  {
    Eigen::VectorXd tau=additive_torque;
    if (std::abs(Dq(m_component_joint_number))<m_Dq_threshold)
    {
      // static condition
      if (std::abs(additive_torque(m_component_joint_number)) <= m_nominal_parameters(0)) 
        tau(m_component_joint_number)=0;
      else if (additive_torque(m_component_joint_number)>m_nominal_parameters(0))
        tau(m_component_joint_number)-=m_nominal_parameters(0);
      else if (additive_torque(m_component_joint_number)<-m_nominal_parameters(0))
        tau(m_component_joint_number)+=m_nominal_parameters(0);
    }
    else 
    {
      tau(m_component_joint_number)+= m_regressor(m_component_joint_number,0)*m_nominal_parameters(0);
    }
    return tau;
  };
  
  
  virtual Eigen::MatrixXd getRegressor(const Eigen::Ref<Eigen::VectorXd>& q,  const Eigen::Ref<Eigen::VectorXd>& Dq, const Eigen::Ref<Eigen::VectorXd>& DDq)
  {
    computeRegressor(Dq);
    return m_regressor;
  }
  
  bool setParameters(const Eigen::Ref<Eigen::VectorXd>& parameters)
  {
//     if (parameters.rows() != m_nominal_parameters.rows() )
//     {
//       ROS_WARN("dimensions mismatch between new parameters and the nominal one");
//       return false;
//     }
    m_nominal_parameters=parameters;
    
    m_parameters_map.at("coloumb")=m_nominal_parameters(0);
    m_parameters_map.at("first_order_viscous")=m_nominal_parameters(1);
    m_parameters_map.at("second_order_viscous")=m_nominal_parameters(2);
    return true;
  }
  
};

}




#endif