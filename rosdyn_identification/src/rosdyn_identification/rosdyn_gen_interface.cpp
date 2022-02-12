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

#include <string>
#include <vector>
#include <math.h>
#include <Eigen/Core>
#include <pluginlib/class_list_macros.h>
#include <name_sorting/sort_trajectories.h>
#include <rosdyn_identification/rosdyn_gen_interface.h>
#include <boost/bind.hpp>

PLUGINLIB_EXPORT_CLASS(rosdyn::MetoGenInterfaceNodelet, nodelet::Nodelet)

namespace rosdyn
{

void MetoGenInterfaceNodelet::onInit()
{
  std::vector<std::string> args = getMyArgv();
  m_stop = false;

  m_meto_trj_opt_as.reset(new actionlib::SimpleActionServer<rosdyn_identification_msgs::MetoTrjGenAction>(getNodeHandle(), "meto_gen_trajectory",
                          boost::bind(&MetoGenInterfaceNodelet::metoTrjGenCB, this, _1), false));

  m_main_thread  = std::thread(&rosdyn::MetoGenInterfaceNodelet::main, this);
  m_meto_trj_opt_as->start();
}

MetoGenInterfaceNodelet::~MetoGenInterfaceNodelet()
{
  m_stop = true;
  m_main_thread.join();
}

void MetoGenInterfaceNodelet::metoTrjGenCB(const rosdyn_identification_msgs::MetoTrjGenGoalConstPtr& goal)
{
  ROS_INFO("MetoGenInterface Callback");

  rosdyn_identification_msgs::MetoTrjGenResult m_result_;
  rosdyn_identification_msgs::MetoTrjGenFeedback m_feedback_;

  // **********************************************************************************************
  // Initialization of MOVEIT
  // **********************************************************************************************

  std::string group_name_     = goal->move_group_name;
  std::string opt_type_       = goal->type;
  m_trj_namespace             = goal->trj_namespace;

  std::string urdf_param_ = "";
  if (!getNodeHandle().getParam(std::string(std::string(m_namespace) + "/urdf_param").c_str(), urdf_param_))
  {
    ROS_INFO("Impossible to find %s\n", std::string(std::string(m_namespace) + "/urdf_param. Use robot_description as default value").c_str());
  }
  std::string robot_description;
  if (!getNodeHandle().getParam(urdf_param_, robot_description))
  {
    ROS_ERROR("Impossible to find robot_description");
    m_result_.result = rosdyn_identification_msgs::MetoTrjGenResult::GENERIC_ERROR;
    m_meto_trj_opt_as->setAborted(m_result_);
    return;
  }


  m_max_speed = 0.0;
  m_max_acc = 0.0;
  if (!getNodeHandle().getParam(std::string(std::string(m_namespace) + "/speed_scaling").c_str(), m_max_speed))
  {
    ROS_ERROR("Impossible to find %s\n", std::string(std::string(m_namespace) + "/speed_scaling").c_str());
    m_result_.result = rosdyn_identification_msgs::MetoTrjGenResult::GENERIC_ERROR;
    m_meto_trj_opt_as->setAborted(m_result_);
    return;
  }
  if (!getNodeHandle().getParam(std::string(std::string(m_namespace) + "/acceleration_scaling").c_str(), m_max_acc))
  {
    ROS_ERROR("Impossible to find %s\n", std::string(std::string(m_namespace) + "/acceleration_scaling").c_str());
    m_result_.result = rosdyn_identification_msgs::MetoTrjGenResult::GENERIC_ERROR;
    m_meto_trj_opt_as->setAborted(m_result_);
    return;
  }

  double stage1_duration = 0;
  if (!getNodeHandle().getParam(std::string(std::string(m_namespace) + "/opt_cfg/stage1_duration").c_str(), stage1_duration))
  {
    ROS_ERROR("Impossible to find %s\n", std::string(std::string(m_namespace) + "/opt_cfg/stage1_duration").c_str());
    m_result_.result = rosdyn_identification_msgs::MetoTrjGenResult::GENERIC_ERROR;
    m_meto_trj_opt_as->setAborted(m_result_);
    return;
  }


  int pnt_of_stage2_ = 0;
  if (!getNodeHandle().getParam(std::string(std::string(m_namespace) + "/opt_cfg/region_stage2").c_str(), pnt_of_stage2_))
  {
    ROS_ERROR("Impossible to find %s\n", std::string(std::string(m_namespace) + "/opt_cfg/region_stage2").c_str());
    m_result_.result = rosdyn_identification_msgs::MetoTrjGenResult::GENERIC_ERROR;
    m_meto_trj_opt_as->setAborted(m_result_);
    return;
  }

  int npoints_per_region;
  if (!getNodeHandle().getParam(std::string(std::string(m_namespace) + "/opt_cfg/point_per_region").c_str(), npoints_per_region))
  {
    ROS_ERROR("Impossible to find %s\n", std::string(std::string(m_namespace) + "/opt_cfg/point_per_region").c_str());
    m_result_.result = rosdyn_identification_msgs::MetoTrjGenResult::GENERIC_ERROR;
    m_meto_trj_opt_as->setAborted(m_result_);
    return;
  }
  int ntrials = 10;
  if (!getNodeHandle().getParam(std::string(std::string(m_namespace) + "/opt_cfg/trials").c_str(), ntrials))
  {
    ROS_ERROR("Impossible to find %s\n", std::string(std::string(m_namespace) + "/opt_cfg/trials").c_str());
    m_result_.result = rosdyn_identification_msgs::MetoTrjGenResult::GENERIC_ERROR;
    m_meto_trj_opt_as->setAborted(m_result_);
    return;
  }
  m_npoints_per_region = npoints_per_region;


  robot_model_loader::RobotModelLoader robot_model_loader(urdf_param_);
  m_kinematic_model = robot_model_loader.getModel();
  m_move_group.reset(new moveit::planning_interface::MoveGroupInterface(group_name_));

  m_move_group->setMaxVelocityScalingFactor(m_max_speed);
  m_move_group->setMaxAccelerationScalingFactor(m_max_acc);

  m_planning_scene.reset(new planning_scene::PlanningScene(m_kinematic_model));

  m_collision_request.group_name = group_name_;
  m_collision_request.contacts = true;
  m_collision_request.max_contacts = 100;
  m_collision_request.max_contacts_per_pair = 1;
  m_collision_request.verbose = false;
  m_collision_request.distance = true;
  m_collision_request.cost = true;


  m_meto_par_estim.reset(new rosdyn::MetoParEstim(getNodeHandle(),
                         robot_description));


  // **********************************************************************************************
  // STAGE 1: INERTIAL Indetification Stage
  // **********************************************************************************************
  trajectory_msgs::JointTrajectory opt_trj;

  if (stage1_duration <= 0)
  {
    ROS_ERROR("Stage 1 duration should be positive");
    m_result_.result = rosdyn_identification_msgs::MetoTrjGenResult::GENERIC_ERROR;
    m_meto_trj_opt_as->setAborted(m_result_);
    return;
  }
  m_feedback_.header.stamp = ros::Time::now();
  m_feedback_.status = rosdyn_identification_msgs::MetoTrjGenFeedback::STEP_1_GEN;
  m_meto_trj_opt_as->publishFeedback(m_feedback_);


  ros::Duration duration(stage1_duration);
  double opt_cost = std::numeric_limits< double >::infinity();

  for (int itrial = 0; itrial < ntrials; itrial++)
  {
    trajectory_msgs::JointTrajectory trj;
    if (firstStageRandomTrajectory(duration, trj))
    {
      double sampling_time = trj.points.back().time_from_start.toSec() / 500;
      double cond_number = checkConditionNumber(trj, sampling_time);
      if (cond_number < opt_cost)
      {
        ROS_INFO("new cost %f trj duration=%f", cond_number, trj.points.back().time_from_start.toSec());
        opt_cost = cond_number;
        opt_trj = trj;
      }
    }
  }

  if (std::isinf(opt_cost))
  {
    m_result_.result = rosdyn_identification_msgs::MetoTrjGenResult::GENERIC_ERROR;
    m_meto_trj_opt_as->setAborted(m_result_);
    return;
  }


  if (std::isinf(opt_cost))
  {
    m_result_.result = rosdyn_identification_msgs::MetoTrjGenResult::GENERIC_ERROR;
    m_meto_trj_opt_as->setAborted(m_result_);
    return;
  }




  if (pnt_of_stage2_ > 0)
  {
    // **********************************************************************************************
    // STAGE 2: FRICTION and MASSES Identification Stage
    // **********************************************************************************************
    m_feedback_.header.stamp = ros::Time::now();
    m_feedback_.status = rosdyn_identification_msgs::MetoTrjGenFeedback::STEP_2_GEN;
    m_meto_trj_opt_as->publishFeedback(m_feedback_);

    trajectory_msgs::JointTrajectory second_stage_opt_trj;
    double opt_cost = std::numeric_limits< double >::infinity();

    for (int itrial = 0; itrial < ntrials; itrial++)
    {
      trajectory_msgs::JointTrajectory trj;
      if (secondStageRandomTrajectory(pnt_of_stage2_, trj, opt_trj.points.back().positions))
      {
        double sampling_time = trj.points.back().time_from_start.toSec() / 500;
        double cond_number = checkConditionNumber(trj, sampling_time);
        if (cond_number < opt_cost)
        {
          ROS_INFO("new cost %f trj duration=%f", cond_number, trj.points.back().time_from_start.toSec());
          opt_cost = cond_number;
          second_stage_opt_trj = trj;
        }
      }
    }

    if (std::isinf(opt_cost))
    {
      m_result_.result = rosdyn_identification_msgs::MetoTrjGenResult::GENERIC_ERROR;
      m_meto_trj_opt_as->setAborted(m_result_);
      return;
    }


    ROS_INFO("MetoGenInterface Callback: Step 2 completed! ");
    trajectory_processing::append_trajectories(opt_trj, second_stage_opt_trj);
  }

  if (!trajectory_processing::setTrajectoryToParam(getNodeHandle(), m_trj_namespace, opt_trj))
  {
    m_result_.result = rosdyn_identification_msgs::MetoTrjGenResult::GENERIC_ERROR;
    m_meto_trj_opt_as->setAborted(m_result_);
    return;
  }

  std::vector<std::string> list_of_trjs;
  if (!getNodeHandle().getParam("list_of_trajectories", list_of_trjs))
    list_of_trjs.clear();

  bool override_trj = false;
  for (const std::string& trj_n : list_of_trjs)
    if (!trj_n.compare(m_trj_namespace))
      override_trj = true;
  if (!override_trj)
    list_of_trjs.push_back(m_trj_namespace);

  getNodeHandle().setParam("list_of_trajectories", list_of_trjs);
  m_result_.result = rosdyn_identification_msgs::MetoTrjGenResult::SUCCESSFUL;
  m_meto_trj_opt_as->setSucceeded(m_result_);


  return;
}


bool MetoGenInterfaceNodelet::firstStageRandomTrajectory(ros::Duration duration, trajectory_msgs::JointTrajectory& trj)
{
  if (!m_kinematic_model)
  {
    ROS_ERROR("no kinematic model");
    return false;
  }
  if (!m_move_group)
  {
    ROS_ERROR("no move group");
    return false;
  }
  moveit::core::RobotState start_state(m_kinematic_model);

  bool success = false;
  while (ros::ok() && !success)
  {
    start_state.setToRandomPositions();
    success = !checkCollision(start_state);
  }
  if (!success)
    return false;
  m_move_group->setStartState(start_state);

  int trial = 0;
  moveit::core::RobotState target_state(m_kinematic_model);
  int fail = 0;

  while (ros::ok() && (trial < 200) && (fail < 5))
  {
    if (trj.points.size() > 0)
      ROS_FATAL("trial %d, fails=%d, trj time=%f, goal=%f", trial, fail, trj.points.back().time_from_start.toSec(), duration.toSec());
    target_state.setToRandomPositions();
    if (checkCollision(target_state))
    {
      trial++;
      continue;
    }
    trial = 0;
    m_move_group->setStartState(start_state);
    m_move_group->setJointValueTarget(target_state);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    double max_speed = m_max_speed * (0.8 * std::rand() + 0.2 * RAND_MAX) / (RAND_MAX);
    double max_acc   = m_max_acc   * (0.8 * std::rand() + 0.2 * RAND_MAX) / (RAND_MAX);
    m_move_group->setMaxVelocityScalingFactor(max_speed);
    m_move_group->setMaxAccelerationScalingFactor(max_acc);

    if (m_move_group->plan(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      fail++;
      continue;
    }

    if (trj.points.size() > 0)
      trajectory_processing::append_trajectories(trj, my_plan.trajectory_.joint_trajectory);
    else
      trj = my_plan.trajectory_.joint_trajectory;

    start_state = target_state;
    if (trj.points.back().time_from_start > duration)
      return true;
  }
  if (fail >= 5)
    ROS_ERROR("failed due to 5 consecutive planing fails");
  else
    ROS_ERROR("failed because unable to find a random valid point");
  return false;
}

bool MetoGenInterfaceNodelet::secondStageRandomTrajectory(const unsigned int& num_of_regions, trajectory_msgs::JointTrajectory& trj, std::vector<double>& initial_joint_conf)
{
  std::map<double, std::shared_ptr<moveit::core::RobotState>> region_seeds;
  for (unsigned int itrial = 0; itrial < 5 * num_of_regions; itrial++)
  {
    bool success = false;
    moveit::core::RobotState seed_state(m_kinematic_model);

    for (unsigned int random_point = 0; random_point < 20; random_point++)
    {
      seed_state.setToRandomPositions();
      if (!checkCollision(seed_state))
      {
        success = true;
        break;
      }
    }
    if (!success)
    {
      ROS_ERROR("unable to find seeds");
      return false;
    }

    std::vector<double> jpos;
    seed_state.copyJointGroupPositions(m_move_group->getName(), jpos);
    Eigen::MatrixXd q(1,  jpos.size());
    Eigen::MatrixXd qp(1, jpos.size());
    Eigen::MatrixXd qpp(1, jpos.size());
    for (unsigned int idx = 0; idx < jpos.size(); idx++)
      q(0, idx) = jpos.at(idx);
    qp.setZero();
    qpp.setZero();

    Eigen::MatrixXd regr = m_meto_par_estim->getTrajectoryRegressor(q, qp, qpp);

    std::pair<double, std::shared_ptr<moveit::core::RobotState>> p(-regr.norm(), std::shared_ptr<moveit::core::RobotState>(new moveit::core::RobotState(seed_state)));
    region_seeds.insert(p);
  }


  std::map<double, std::shared_ptr<moveit::core::RobotState>>::iterator it = region_seeds.begin();
  trj.points.clear();
  trj.joint_names = m_kinematic_model->getJointModelNames();

  moveit::core::RobotState start_state(m_kinematic_model);
  start_state.setJointGroupPositions(m_move_group->getName(), initial_joint_conf);

  m_move_group->setStartState(start_state);
  for (unsigned int ireg = 0; ireg < num_of_regions; ireg++)
  {
    ROS_DEBUG("region seed %u", ireg);

    moveit::core::RobotState target_state(*(it->second));
    m_move_group->setJointValueTarget(target_state);


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if (m_move_group->plan(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_WARN("region seed %u is unreachable, skip", ireg);
      it++;
      continue;
    }

    if (trj.points.size() > 0)
      trajectory_processing::append_trajectories(trj, my_plan.trajectory_.joint_trajectory);
    else
      trj = my_plan.trajectory_.joint_trajectory;

    start_state = *(it->second);
    m_move_group->setStartState(start_state);


    unsigned int region_points = 0;
    for (unsigned int ipnt = 0; ipnt < 5 * m_npoints_per_region; ipnt++)
    {
      moveit::core::RobotState target_state(*(it->second));
      target_state.setToRandomPositionsNearBy(m_kinematic_model->getJointModelGroup(m_move_group->getName()), *(it->second), 0.1);
      if (checkCollision(target_state))
        continue;

      m_move_group->setJointValueTarget(target_state);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;

      double max_speed = m_max_speed * (0.8 * std::rand() + 0.2 * RAND_MAX) / (RAND_MAX);
      double max_acc   = m_max_acc   * (0.8 * std::rand() + 0.2 * RAND_MAX) / (RAND_MAX);
      m_move_group->setMaxVelocityScalingFactor(max_speed);
      m_move_group->setMaxAccelerationScalingFactor(max_acc);

      if (m_move_group->plan(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        continue;
      }

      if (trj.points.size() > 0)
        trajectory_processing::append_trajectories(trj, my_plan.trajectory_.joint_trajectory);
      else
        trj = my_plan.trajectory_.joint_trajectory;

      m_move_group->setStartState(target_state);
      if (++region_points > m_npoints_per_region)
        break;
    }

    it++;
  }
  return (trj.points.size() > 0);
}


bool MetoGenInterfaceNodelet::checkCollision(moveit::core::RobotState& state)
{
  collision_detection::CollisionResult collision_result;
  state.update();
  m_planning_scene->checkCollision(m_collision_request, collision_result, state);
  return collision_result.collision;
}


double MetoGenInterfaceNodelet::checkConditionNumber(const trajectory_msgs::JointTrajectory& trj, double sampling_time)
{
  trajectory_processing::SplineInterpolator sp;
  if (!sp.setTrajectory(trj))
    return std::numeric_limits< double >::infinity();

  trajectory_msgs::JointTrajectory resampled_trj;
  if (!sp.resampleTrajectory(sampling_time, resampled_trj))
    return std::numeric_limits< double >::infinity();

  unsigned int nax = resampled_trj.points.at(0).positions.size();
  Eigen::MatrixXd q(resampled_trj.points.size(),   nax);
  Eigen::MatrixXd qp(resampled_trj.points.size(),  nax);
  Eigen::MatrixXd qpp(resampled_trj.points.size(), nax);
  for (unsigned int ipnt = 0; ipnt < resampled_trj.points.size(); ipnt++)
  {
    for (unsigned int iax = 0; iax < nax; iax++)
    {
      q(ipnt, iax)   = resampled_trj.points.at(ipnt).positions.at(iax);
      qp(ipnt, iax)  = resampled_trj.points.at(ipnt).velocities.at(iax);
      qpp(ipnt, iax) = resampled_trj.points.at(ipnt).accelerations.at(iax);
    }
  }
  m_meto_par_estim->getTrajectoryRegressor(q, qp, qpp);
  return m_meto_par_estim->getConditionNumber();
}



void MetoGenInterfaceNodelet::main()
{
  ROS_INFO("Starting THREAD MetoGenInterface");

  m_namespace = "meto_cfg";

  while ((ros::ok()) && (!m_stop))
  {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  ROS_INFO("End THREAD MetoGenInterface");

  return;
}

}  // namespace rosdyn

