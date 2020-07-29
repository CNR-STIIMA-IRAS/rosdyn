/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once  // NOLINT(build/header_guard)
#include <string>
#include <vector>

#if ROS_VERSION_MINIMUM(1, 14, 1)
# include <memory>
namespace shared_ptr_namespace = std;
#else
# include <boost/concept_check.hpp>
# include <boost/graph/graph_concepts.hpp>
# include <boost/enable_shared_from_this.hpp>
namespace shared_ptr_namespace = boost;
#endif
# include <Eigen/Core>
# include <ros/ros.h>

namespace rosdyn
{
class ComponentBase
{
protected:
  std::string m_type;

  // Name of the robot
  std::string m_robot_name;

  // Name of the Joint where the component is applied
  std::string m_component_joint_name;

  // Name of the joints of the tree
  std::vector<std::string>  m_joint_names;

  // Number of Joints
  unsigned int m_joints_number;

  // Number of the joint component in the list of joint names
  unsigned int m_component_joint_number;

  Eigen::VectorXd m_torques;
  Eigen::MatrixXd m_regressor;
  Eigen::VectorXd m_nominal_parameters;
  ros::NodeHandle m_nh;

  std::map<std::string, double> m_parameters_map;
  std::map<std::string, double> m_constants_map;

  void loadParametersAndConstants()
  {
    if (!m_nh.getParam(m_robot_name + "/" + m_component_joint_name + "/" + m_type + "/coefficients", m_parameters_map))
    {
      ROS_DEBUG_STREAM(m_component_joint_name + "/" + m_component_joint_name + "/" + m_type + "/coefficients NOT FOUND");  // NOLINT(whitespace/line_length)
      m_parameters_map.clear();
    }
    if (!m_nh.getParam(m_robot_name + "/" + m_component_joint_name + "/" + m_type + "/constants", m_constants_map))
    {
      ROS_DEBUG_STREAM(m_component_joint_name + "/" + m_component_joint_name + "/" + m_type + "/constants NOT FOUND");
      m_constants_map.clear();
    }
  }

public:
  ComponentBase(const std::string& joint_name, const std::string& robot_name, const ros::NodeHandle& nh)
  {
    m_component_joint_name = joint_name;
    m_robot_name = robot_name;
    m_nh = nh;
    if (!nh.getParam(robot_name + "/joint_names", m_joint_names))
    {
      throw std::invalid_argument("PARAMETER '" + robot_name + "/joint_names' NOT FOUND");
    }
    m_joints_number = m_joint_names.size();

    for (m_component_joint_number = 0; m_component_joint_number < m_joints_number; m_component_joint_number++)
      if (!m_joint_names.at(m_component_joint_number).compare(m_component_joint_name))
        break;

    if (m_component_joint_number == m_joints_number)
      throw std::invalid_argument("Component Joint name '" + m_component_joint_name + "' is not a elemente of '" + robot_name + "/joint_names'");

    m_torques.resize(m_joints_number);
    m_torques.setZero();
  }

  virtual Eigen::VectorXd getTorque(const Eigen::Ref<Eigen::VectorXd>& q,  const Eigen::Ref<Eigen::VectorXd>& Dq, const Eigen::Ref<Eigen::VectorXd>& DDq) = 0;

  virtual Eigen::VectorXd getAdditiveTorque(const Eigen::Ref<Eigen::VectorXd>& q,  const Eigen::Ref<Eigen::VectorXd>& Dq, const Eigen::Ref<Eigen::VectorXd>& DDq)
  {
    return getTorque(q, Dq, DDq);
  }

  virtual Eigen::VectorXd getNonAdditiveTorque(const Eigen::Ref<Eigen::VectorXd>& q,  const Eigen::Ref<Eigen::VectorXd>& Dq, const Eigen::Ref<Eigen::VectorXd>& DDq, const Eigen::Ref<Eigen::VectorXd>& additive_torque)
  {
    Eigen::VectorXd t(m_joints_number);
    t.setZero();
    return t;
  }

  virtual Eigen::MatrixXd getRegressor(const Eigen::Ref<Eigen::VectorXd>& q,  const Eigen::Ref<Eigen::VectorXd>& Dq, const Eigen::Ref<Eigen::VectorXd>& DDq) = 0;

  unsigned int getParametersNumber()
  {
    return m_parameters_map.size();
  }

  Eigen::VectorXd getNominalParameters()
  {
    return m_nominal_parameters;
  }

  std::string getJointName()
  {
    return m_component_joint_name;
  }

  unsigned int getJointNumber()
  {
    return m_component_joint_number;
  }

  virtual bool setParameters(const Eigen::Ref<Eigen::VectorXd>& parameters) = 0;

  void saveParameters()
  {
    m_nh.setParam(m_robot_name + "/" + m_component_joint_name + "/" + m_type + "/coefficients", m_parameters_map);
  }

  std::map<std::string, double> getParametersMap()
  {
    return m_parameters_map;
  }

  bool setParameters(std::map<std::string, double> parameters)
  {
    for (const std::pair<std::string, double>& par : parameters)
    {
      try
      {
        m_parameters_map.at(par.first) = par.second;
      }
      catch (std::out_of_range& ex)
      {
        ROS_ERROR("parameter %s = %f is not a member of the component", par.first.c_str(), par.second);
        return false;
      }
    }
    return false;
  }
};


typedef  shared_ptr_namespace::shared_ptr<rosdyn::ComponentBase> ComponentPtr;
}  // namespace rosdyn

