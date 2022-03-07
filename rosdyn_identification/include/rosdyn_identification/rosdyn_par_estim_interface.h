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


#include <mutex>
#include <thread>
#include <fstream>
#include <iostream>

#include <nodelet/nodelet.h>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <actionlib/server/simple_action_server.h>

#include <rosdyn_identification_msgs/MetoParEstimAction.h>
#include <rosdyn_identification/rosdyn_par_estim.h>

#include <std_srvs/Empty.h>

/*! \file meto_par_estim_interface.h
    \brief A Documented file.

    Details.
*/

namespace rosdyn
{

//! The Nodelet inteface class to \link MetoParEstim \endlink.
/*!
  The class is the Nodelet interface to the base class \link MetoParEstim \endlink .
*/
class MetoParEstimInterfaceNodelet : public nodelet::Nodelet
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Initialization of the ROS infrastructure for Nodelet.
  /*!
    Virtual and must be overridden by subclass. All initialization of the ROS infrastructure must be put into this function.
  */
  virtual void onInit();

protected:
  //! Flag to stop the Nodelet thread
  bool m_stop;

  //! Node parameters' namepace.
  std::string m_namespace;

  //! Temporary string that contains the binary file to be loaded
  std::string m_file_name;
  std::string m_model_name;

  //! Main Nodelet thread for the dynamics parameters estimation.
  std::thread m_main_thread;

  ros::ServiceServer m_save_model_server;

  std::shared_ptr<rosdyn::MetoParEstim> m_estimator;

  //! Action Server.
  /*!
    Action Server of the rosdyn_identification::MetoParEstimAction.
  */
  std::shared_ptr<actionlib::SimpleActionServer<rosdyn_identification_msgs::MetoParEstimAction>> m_meto_par_estim_as;


  //! Main function to enter in an infinite loop.
  /*!

  */
  void main();


  bool saveXmlCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

  //! Callback of the rosdyn_identification::MetoParEstimAction.
  /*!
    \param goal goal of the rosdyn_identification::MetoParEstimAction
  */
  void metoParEstimCB(const rosdyn_identification_msgs::MetoParEstimGoalConstPtr& goal);

  //! A destructor of the class \link MetoParEstimInterfaceNodelet \endlink.
  /*!
  */
  ~MetoParEstimInterfaceNodelet();
};


}  // namespace rosdyn
