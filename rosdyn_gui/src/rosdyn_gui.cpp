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
#include <rosdyn_gui.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rosdyn_gui::IdentificationGui, rviz::Panel)

#include <string>
#include <vector>

namespace rosdyn_gui
{


IdentificationGui::IdentificationGui(QWidget* parent):
  Panel(parent)
{
  m_grid_layout = new QGridLayout(this);

  m_generate_trajectories_btn = new QPushButton();
  m_generate_trajectories_btn->setText("1) Generate Trajectory");
  connect(m_generate_trajectories_btn, SIGNAL(clicked()), this, SLOT(generateTrjCallback()));
  m_grid_layout->addWidget(m_generate_trajectories_btn, 0, 0, 1, 1);
  /*
   m_load_trj_btn = new QPushButton();
   m_load_trj_btn->setText("Load Trajectory");
   connect(m_load_trj_btn, SIGNAL(clicked()), this, SLOT(loadTrjCallback()));
   m_grid_layout->addWidget( m_load_trj_btn, 0, 1, 1, 1);

   m_save_trj_btn = new QPushButton();
   m_save_trj_btn->setText("Save Trajectory");
   connect(m_save_trj_btn, SIGNAL(clicked()), this, SLOT(saveTrjCallback()));
   m_grid_layout->addWidget( m_save_trj_btn, 0, 2, 1, 1);*/

  m_simulate_trajectories_btn = new QPushButton();
  m_simulate_trajectories_btn->setText("2) Simulate Trajectory");
  connect(m_simulate_trajectories_btn, SIGNAL(clicked()), this, SLOT(simulateTrjCallback()));
  m_grid_layout->addWidget(m_simulate_trajectories_btn, 1, 0, 1, 1);

  m_execute_trajectories_btn = new QPushButton();
  m_execute_trajectories_btn->setText("3) Execute Trajectory");
  connect(m_execute_trajectories_btn, SIGNAL(clicked()), this, SLOT(executeTrjCallback()));
  m_grid_layout->addWidget(m_execute_trajectories_btn, 2, 0, 1, 1);


  m_execute_trajectories_btn->setEnabled(false);

  m_parameter_estimation_btn = new QPushButton();
  m_parameter_estimation_btn->setText("4) Estimate Model");
  connect(m_parameter_estimation_btn, SIGNAL(clicked()), this, SLOT(modelEstimationCallback()));
  m_grid_layout->addWidget(m_parameter_estimation_btn, 3, 0, 1, 1);


  trj_label = new QLabel(tr("Select trajectory"));
  m_grid_layout->addWidget(trj_label,        0, 1, 1, 1);
  m_list_trjs_combobox = new QComboBox();
  updateTrajectoriesList();
  connect(m_list_trjs_combobox, SIGNAL(currentIndexChanged(QString)), this, SLOT(newTrajectory(QString)));
  m_grid_layout->addWidget(m_list_trjs_combobox, 1, 1, 1, 1);

  std::vector<std::string> executed_trjs;
  if (!m_nh.getParam("list_of_trajectories", executed_trjs))
  {
    m_simulate_trajectories_btn->setEnabled(false);
    m_list_trjs_combobox->setEnabled(false);
    trj_label->setEnabled(false);
    m_parameter_estimation_btn->setEnabled(false);
  }
  m_save_model_btn = new QPushButton();
  m_save_model_btn->setText("Save Model");
  connect(m_save_model_btn, SIGNAL(clicked()), this, SLOT(saveModelCallback()));
  m_grid_layout->addWidget(m_save_model_btn, 2, 1, 1, 1);
  m_save_model_btn->setEnabled(false);

  m_setup_btn = new QPushButton();
  m_setup_btn->setText("Advanced settings");
  connect(m_setup_btn, SIGNAL(clicked()), this, SLOT(setupCallback()));
  m_grid_layout->addWidget(m_setup_btn, 3, 1, 1, 1);

  meto_gen_ac_.reset(
    new actionlib::SimpleActionClient<rosdyn_identification_msgs::MetoTrjGenAction>(
      "/meto_gen_trajectory",
      true));
  meto_estim_ac_.reset(
    new actionlib::SimpleActionClient<rosdyn_identification_msgs::MetoParEstimAction>("/meto_param_estimation",
        true));
  meto_exec_ac_.reset(
    new actionlib::SimpleActionClient<moveit_planning_helper::ExecuteTrajectoryFromParamAction>(
      "/execute_trajectories_from_param",
      true));
  m_save_model_client = m_nh.serviceClient<std_srvs::Empty>("meto_save_model");
}

IdentificationGui::~IdentificationGui()
{
}

void IdentificationGui::onInitialize()
{
}


void IdentificationGui::generateTrjCallback()
{
  rosdyn_identification_msgs::MetoTrjGenGoal meto_gen_goal_;
  std::string group_name;
  if (!m_nh.getParam("group_name", group_name))
  {
    ROS_ERROR("group_name is not specified");
    return;
  }
  std::string type;
  if (!m_nh.getParam("meto_cfg/type", type))
  {
    type = "arm";
  }
  std::string trajectory_namespace;
  if (!m_nh.getParam("meto_cfg/trajectory_namespace", trajectory_namespace))
  {
    ROS_ERROR("meto_cfg/trajectory_namespace is not specified");
    return;
  }
  meto_gen_goal_.move_group_name  = group_name;
  meto_gen_goal_.type             = type;
  meto_gen_goal_.trj_namespace    = trajectory_namespace;

  if (!meto_gen_ac_->waitForServer(ros::Duration(1)))
  {
    ROS_ERROR("no exection_server");
    return;
  }


  meto_gen_ac_->sendGoal(meto_gen_goal_, boost::bind(&IdentificationGui::generationDoneCb, this, _1, _2));
  m_generate_trajectories_btn->setText("Generating....");
}
void IdentificationGui::loadTrjCallback()
{
  updateTrajectoriesList();
}
void IdentificationGui::saveTrjCallback()
{
}

void IdentificationGui::simulateTrjCallback()
{
  moveit_planning_helper::ExecuteTrajectoryFromParamGoal goal;
  std::string group_name;
  if (!m_nh.getParam("group_name", group_name))
  {
    ROS_ERROR("group_name is not specified");
    return;
  }
  goal.group_name = group_name;
  goal.simulation = true;

  std::string trajectory_name;
  if (!m_nh.getParam("meto_cfg/trajectory_namespace", trajectory_name))
  {
    ROS_ERROR("meto_cfg/trajectory_namespace is not specified");
    return;
  }
  goal.trajectory_names.push_back(trajectory_name);

  meto_exec_ac_->sendGoal(goal, boost::bind(&IdentificationGui::executionDoneCb, this, _1, _2));
  m_simulate_trajectories_btn->setText("Simulating...");
}

void IdentificationGui::executeTrjCallback()
{
  moveit_planning_helper::ExecuteTrajectoryFromParamGoal goal;
  std::string group_name;
  if (!m_nh.getParam("group_name", group_name))
  {
    ROS_ERROR("group_name is not specified");
    return;
  }
  goal.group_name = group_name;
  goal.simulation = false;

  std::string trajectory_name;
  if (!m_nh.getParam("meto_cfg/trajectory_name", trajectory_name))
  {
    ROS_ERROR("meto_cfg/trajectory_name is not specified");
    return;
  }
  goal.trajectory_names.push_back(trajectory_name);

  meto_exec_ac_->sendGoal(goal, boost::bind(&IdentificationGui::executionDoneCb, this, _1, _2));
  m_execute_trajectories_btn->setText("Executing...");
}
void IdentificationGui::modelEstimationCallback()
{
  rosdyn_identification_msgs::MetoParEstimGoal goal;
  if (!m_nh.getParam("meto_cfg/trajectory_namespace", goal.trj_namespace))
  {
    ROS_ERROR("meto_cfg/trajectory_namespace is not specified");
    return;
  }
  if (!m_nh.getParam("group_name", goal.group_name))
  {
    ROS_ERROR("group_name is not specified");
    return;
  }
  m_parameter_estimation_btn->setText("Computing...");
  meto_estim_ac_->sendGoal(goal, boost::bind(&IdentificationGui::estimationDoneCb, this, _1, _2));
}

void IdentificationGui::saveModelCallback()
{
  if (!m_save_model_client.exists())
    m_save_model_btn->setText("Save server is down");

  std_srvs::Empty srv;
  if (!m_save_model_client.call(srv))
    m_save_model_btn->setText("Save server gives an error");
}
void IdentificationGui::setupCallback()
{
  m_setup_popup = new TabDialog(m_nh, this);
  m_setup_popup->exec();
  m_setup_popup->adjustSize();
}


void IdentificationGui::updateTrajectoriesList()
{
  m_list_trjs_combobox->clear();
  std::vector<std::string> executed_trjs;
  if (!m_nh.getParam("list_of_trajectories", executed_trjs))
  {
    ROS_DEBUG("NO list_of_trajectories specified");
    return;
  }
  for (const std::string& str : executed_trjs)
    m_list_trjs_combobox->addItem(str.c_str());
}


void IdentificationGui::newTrajectory(QString select_trj)
{
  std::string trj(select_trj.toUtf8().constData());

  if (!trj.compare("all"))
    m_nh.deleteParam("meto_cfg/trajectory_name");
  else
    m_nh.setParam("meto_cfg/trajectory_name", trj);
  ROS_FATAL("%s", trj.c_str());
}

void IdentificationGui::generationDoneCb(const actionlib::SimpleClientGoalState& state,
    const rosdyn_identification_msgs::MetoTrjGenResultConstPtr& result)
{
  m_generate_trajectories_btn->setText("1) Generate Trajectory");
  updateTrajectoriesList();
  m_simulate_trajectories_btn->setEnabled(true);
  m_list_trjs_combobox->setEnabled(true);
  trj_label->setEnabled(true);
}

void IdentificationGui::executionDoneCb(const actionlib::SimpleClientGoalState& state,
                                        const moveit_planning_helper::ExecuteTrajectoryFromParamResultConstPtr& result)
{
  m_simulate_trajectories_btn->setText("2) Simulate Trajectory");
  m_execute_trajectories_btn->setText("3) Execute Trajectory");
  if (m_execute_trajectories_btn->isEnabled())
    m_parameter_estimation_btn->setEnabled(true);
  m_execute_trajectories_btn->setEnabled(true);
}

void IdentificationGui::estimationDoneCb(const actionlib::SimpleClientGoalState& state,
    const rosdyn_identification_msgs::MetoParEstimResultConstPtr& result)
{
  m_save_model_btn->setEnabled(true);
  ROS_INFO_STREAM("result\n" << *result);
  // m_results_popup = new ResultsTabDialog(m_nh,result,this);
  // m_results_popup->setWindowModality(Qt::NonModal);
  // m_results_popup->adjustSize();
  // m_results_popup->exec();
  m_parameter_estimation_btn->setText("4) Estimate Model");
}

}  // namespace rosdyn_gui



