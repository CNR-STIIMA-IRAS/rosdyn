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

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>


#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <rosdyn_identification_msgs/MetoParEstimAction.h>
#include <rosdyn_identification_msgs/MetoTrjGenAction.h>
#include <moveit_planning_helper/ExecuteTrajectoryFromParamAction.h>

#include "rviz/panel.h"
#include "rviz/visualization_manager.h"

#include <QVBoxLayout>
#include <QPushButton>
#include <QRadioButton>
#include <QGroupBox>
#include <QComboBox>
#include <QDialog>
#include <setup_popup.h>
#include <dyn_components.h>
#include <results_popup.h>
#include <string>
#include <vector>

// class QPushButton;
namespace rosdyn_gui
{



class IdentificationGui: public rviz::Panel
{
  Q_OBJECT
public:
  explicit IdentificationGui(QWidget* parent = 0);
  virtual ~IdentificationGui();
  virtual void onInitialize();
  /** @brief Load the given html file. */


protected Q_SLOTS:
  void callback(unsigned int num_btn);

  void generateTrjCallback();
  void saveTrjCallback();
  void loadTrjCallback();

  void simulateTrjCallback();
  void executeTrjCallback();
  void newTrajectory(QString select_trj);

  void modelEstimationCallback();
  void saveModelCallback();
  void setupCallback();

protected:
  ros::NodeHandle m_nh;

  QGridLayout* m_grid_layout;

  QPushButton* m_generate_trajectories_btn;
  QPushButton* m_save_trj_btn;
  QPushButton* m_load_trj_btn;

  QPushButton* m_simulate_trajectories_btn;
  QPushButton* m_execute_trajectories_btn;
  QComboBox*   m_list_trjs_combobox;

  QPushButton* m_parameter_estimation_btn;
  QPushButton* m_save_model_btn;
  QPushButton* m_setup_btn;
  TabDialog* m_setup_popup;
  ResultsTabDialog* m_results_popup;

  std::shared_ptr<actionlib::SimpleActionClient<rosdyn_identification_msgs::MetoTrjGenAction>> meto_gen_ac_;
  std::shared_ptr<actionlib::SimpleActionClient<moveit_planning_helper::ExecuteTrajectoryFromParamAction>> meto_exec_ac_;  // NOLINT(whitespace/line_length)

  std::shared_ptr<actionlib::SimpleActionClient<rosdyn_identification_msgs::MetoParEstimAction>> meto_estim_ac_;
  ros::ServiceClient m_save_model_client;
  QLabel* trj_label;

  void updateTrajectoriesList();
  void generationDoneCb(const actionlib::SimpleClientGoalState& state, const rosdyn_identification_msgs::MetoTrjGenResultConstPtr& result);  // NOLINT(whitespace/line_length)
  void executionDoneCb(const actionlib::SimpleClientGoalState& state, const moveit_planning_helper::ExecuteTrajectoryFromParamResultConstPtr& result);  // NOLINT(whitespace/line_length)
  void estimationDoneCb(const actionlib::SimpleClientGoalState& state, const rosdyn_identification_msgs::MetoParEstimResultConstPtr& result);  // NOLINT(whitespace/line_length)
};

}  // namespace rosdyn_gui


