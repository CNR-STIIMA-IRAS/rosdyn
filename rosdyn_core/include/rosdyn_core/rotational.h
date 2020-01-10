#ifndef __ITIA_DYNAMICS_CORE__FRICTION_ROTATIONAL__
#define __ITIA_DYNAMICS_CORE__FRICTION_ROTATIONAL__

# include <rosdyn_core/base_component.h>
//# include <math>

namespace rosdyn
{


class RotationalFriction: public ComponentBase
{
protected:
  double brk_torque;
  double brk_velocity;
  double viscous_coeff;
  double coulomb_torque;
  double static_scale;
  double static_thr;
  double coulomb_thr;
  
  void computeRegressor(const Eigen::Ref<Eigen::VectorXd>& Dq )
  {
    m_regressor.setZero();
  }
  
public:
  RotationalFriction(const std::string& joint_name, const std::string& robot_name, const ros::NodeHandle& nh): ComponentBase(joint_name, robot_name, nh)
  {
    m_type="friction";
    loadParametersAndConstants();
    
    if (m_constants_map.size()<3)
      throw std::invalid_argument(robot_name+"/" + joint_name + "/friction/constants has wrong dimensions, " + std::to_string(m_constants_map.size()));
    
    if (m_parameters_map.size()<1)
      throw std::invalid_argument(robot_name+"/" + joint_name + "/friction/coefficients has wrong dimensions, " + std::to_string(m_parameters_map.size()));
    
//    m_nominal_parameters.resize(3);
    brk_torque=m_constants_map.at("brkwy_torque");
    brk_velocity=m_constants_map.at("brkwy_velocity");
    coulomb_torque=brk_torque * m_constants_map.at("coulomb_torque");

    viscous_coeff = m_parameters_map.at("viscous_coeff");

    static_scale = sqrt(2 * exp(1)) * (brk_torque - coulomb_torque);
    static_thr = sqrt(2) * brk_velocity;
    coulomb_thr = brk_velocity/10;

    m_regressor.resize(m_joints_number, 1);
    m_regressor.setZero();
    

    
    
  };
    
  virtual Eigen::VectorXd getTorque(const Eigen::Ref<Eigen::VectorXd>& q,  const Eigen::Ref<Eigen::VectorXd>& Dq, const Eigen::Ref<Eigen::VectorXd>& DDq)
  {
    m_torques(m_component_joint_number) = 0;
    return m_torques;
  };
  
  virtual Eigen::VectorXd getAdditiveTorque(const Eigen::Ref< Eigen::VectorXd >& q, const Eigen::Ref< Eigen::VectorXd >& Dq, const Eigen::Ref< Eigen::VectorXd >& DDq)
  {

    m_torques(m_component_joint_number) = 0;
    return m_torques;
  };
  
  virtual Eigen::VectorXd getNonAdditiveTorque(const Eigen::Ref< Eigen::VectorXd >& q, const Eigen::Ref< Eigen::VectorXd >& Dq, const Eigen::Ref< Eigen::VectorXd >& DDq, const Eigen::Ref< Eigen::VectorXd >& additive_torque)
  {
    Eigen::VectorXd tau=additive_torque;
    double vel = Dq(m_component_joint_number);

    double force = viscous_coeff*vel + static_scale*((vel/static_thr)*exp(-((vel/static_thr)*(vel/static_thr)))) + coulomb_torque*tanh(vel/coulomb_thr);
//    std::cout << vel << " " << force << " " << tau(m_component_joint_number) << " " << m_component_joint_number  <<std::endl;


    tau(m_component_joint_number) -=  force;

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
    ROS_INFO_STREAM(m_component_joint_name << ": " << (parameters.transpose()));
    m_parameters_map.at("viscous_coeff")=viscous_coeff;
    return true;
  }
  
};

}




#endif
