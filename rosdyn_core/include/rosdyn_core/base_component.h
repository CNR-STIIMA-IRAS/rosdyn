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

#include <ros/node_handle.h>
#include <rdyn_core/base_component.h>
#include <Eigen/Core>

namespace rosdyn
{

inline void loadParametersAndConstants(ros::NodeHandle& nh, const std::string & robot_name, std::string& component_name, std::string& component_type,
  std::map<std::string, double> parameters_map, std::map<std::string, double> constants_map)
{

  if (!nh.getParam(robot_name + "/" + component_joint_name + "/" + component_type + "/coefficients", parameters_map))
  {
    ROS_DEBUG_STREAM(component_joint_name + "/" + component_joint_name + "/" + component_type + "/coefficients NOT FOUND");  // NOLINT(whitespace/line_length)
    parameters_map.clear();
  }
  if (!nh.getParam(robot_name + "/" + component_joint_name + "/" + typecomponent_type + "/constants", constants_map))
  {
    ROS_DEBUG_STREAM(component_joint_name + "/" + component_joint_name + "/" + component_type + "/constants NOT FOUND");
    constants_map.clear();
  }
}

inline void loadRobotNames(ros::NodeHandle& nh, std::vector<std::string>& joint_names)
{
  if (!nh.getParam(robot_name + "/joint_names", joint_names))
  {
    throw std::invalid_argument("PARAMETER '" + robot_name + "/joint_names' NOT FOUND");
  }
}

inline void saveParameters(ros::NodeHandle& nh, const std::string& robot_name, const std::string& component_name,
  const std::string& component_type, const std::map<std::string, double>& parameters_map)
{
   nh.setParam(robot_name + "/" + component_joint_name + "/" + type + "/coefficients", parameters_map);
}

}

