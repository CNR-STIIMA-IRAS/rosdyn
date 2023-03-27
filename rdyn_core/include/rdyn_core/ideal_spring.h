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
#include <rdyn_core/base_component.h>


namespace rdyn
{

class IdealSpring: public ComponentBase
{
protected:
  double m_elasticity;
  double m_offset;

public:
  IdealSpring(const std::string& joint_name, const std::string& robot_name, const ros::NodeHandle& nh): ComponentBase(joint_name, robot_name, nh)
  {
    m_type = "spring";
    loadParametersAndConstants();

    m_nominal_parameters.resize(2);
    m_elasticity = m_nominal_parameters(0) = m_parameters_map.at("elasticity");
    m_offset = m_nominal_parameters(1) = m_parameters_map.at("offset_effort");

    m_regressor.resize(m_joints_number, 2);
    m_regressor.setZero();
  }

  virtual Eigen::VectorXd getTorque(const Eigen::Ref<Eigen::VectorXd>& q,  const Eigen::Ref<Eigen::VectorXd>& Dq, const Eigen::Ref<Eigen::VectorXd>& DDq)
  {
    maybe_unused(Dq,DDq);
    m_torques(m_component_joint_number) = m_elasticity * q(m_joints_number) + m_offset;
    return m_torques;
  }

  virtual Eigen::MatrixXd getRegressor(const Eigen::Ref<Eigen::VectorXd>& q,  const Eigen::Ref<Eigen::VectorXd>& Dq, const Eigen::Ref<Eigen::VectorXd>& DDq)
  {
    maybe_unused(Dq,DDq);
    m_regressor(m_component_joint_number, 0) = q(m_component_joint_number);
    m_regressor(m_component_joint_number, 1) = 1;
    return m_regressor;
  }

  bool setParameters(const Eigen::Ref<Eigen::VectorXd>& parameters)
  {
    if (parameters.rows() != m_nominal_parameters.rows())
    {
      ROS_WARN("dimensions mismatch between new parameters and the nominal one");
      return false;
    }
    m_nominal_parameters = parameters;

    m_parameters_map.at("elasticity") = m_nominal_parameters(0);
    m_parameters_map.at("offset_effort") = m_nominal_parameters(1);
    return true;
  }
};

}  // namespace rdyn


