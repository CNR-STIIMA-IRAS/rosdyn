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


#include <rosdyn_core/primitives.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosdyn_core/urdf_parser.h>
#include <string>
#include <vector>

#include <thread>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jacobian_speed_test");
  ros::NodeHandle nh;
  ros::Rate rate(1);

  std::string base_frame = "ur10_base_link";
  std::string tool_frame = "ur10_ee_link";

  sensor_msgs::JointState js;
  js.name.resize(6);
  js.position.resize(6);
  js.velocity.resize(6);
  js.name.at(0) = "ur10_shoulder_pan_joint";
  js.name.at(1) = "ur10_shoulder_lift_joint";
  js.name.at(2) = "ur10_elbow_joint";
  js.name.at(3) = "ur10_wrist_1_joint";
  js.name.at(4) = "ur10_wrist_2_joint";
  js.name.at(5) = "ur10_wrist_3_joint";

  urdf::Model model;
  model.initParam("robot_description");

  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  shared_ptr_namespace::shared_ptr<rosdyn::Chain> chain = rosdyn::createChain(model, base_frame, tool_frame, grav);
//  chain->setInputJointsName(js.name);

  unsigned int n_joints = chain->getActiveJointsNumber();
  Eigen::VectorXd q(n_joints);
  q.setZero();
  Eigen::VectorXd Dq(n_joints);
  Dq.setZero();
  Eigen::VectorXd DDq(n_joints);
  DDq.setZero();
  Eigen::VectorXd DDDq(n_joints);
  DDDq.setZero();
  Eigen::VectorXd tau(n_joints);
  tau.setZero();



  Eigen::Affine3d T_base_tool;

  Eigen::Vector6d twist_of_tool_in_base;
  Eigen::Vector6d nonlinacc_twist_of_tool_in_base;
  Eigen::Vector6d linacc_twist_of_tool_in_base;
  Eigen::Vector6d acc_twist_of_tool_in_base;
  Eigen::Vector6d jerk_twist_of_tool_in_base;

  std::vector< Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d> > twists, nonlinacc_twists, linacc_twists, acc_twists, jerk_twists;  // NOLINT(whitespace/line_length)

  Eigen::Matrix6Xd jacobian_of_tool_in_base;
  jacobian_of_tool_in_base.resize(6, chain->getActiveJointsNumber());
  Eigen::MatrixXd joint_inertia;

  double t_null = 0;
  double t_pose_eigen = 0;
  double t_jac_eigen = 0;
  double t_vel_eigen = 0;
  double t_acc_eigen = 0;
  double t_jerk_eigen = 0;
  double t_linacc_eigen = 0;
  double t_nonlinacc_eigen = 0;
  double t_torque_eigen = 0;
  double t_inertia_eigen = 0;
  ros::Time t0;
  int ntrial = 1e4;


  for (int idx = 0; idx < ntrial; idx++)
  {
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();

    t0 = ros::Time::now();
    t_null += (ros::Time::now() - t0).toSec() * 1e6;
    t0 = ros::Time::now();

    T_base_tool = chain->getTransformation(q);
    t_pose_eigen += (ros::Time::now() - t0).toSec() * 1e6;

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = ros::Time::now();
    jacobian_of_tool_in_base = chain->getJacobian(q);
    t_jac_eigen += (ros::Time::now() - t0).toSec() * 1e6;

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = ros::Time::now();
    twists = chain->getTwist(q, Dq);
    t_vel_eigen += (ros::Time::now() - t0).toSec() * 1e6;

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = ros::Time::now();
    linacc_twists = chain->getDTwistLinearPart(q, DDq);
    t_linacc_eigen += (ros::Time::now() - t0).toSec() * 1e6;

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = ros::Time::now();
    nonlinacc_twists = chain->getDTwistNonLinearPart(q, Dq);
    t_nonlinacc_eigen += (ros::Time::now() - t0).toSec() * 1e6;

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = ros::Time::now();
    acc_twists = chain->getDTwist(q, Dq, DDq);
    t_acc_eigen += (ros::Time::now() - t0).toSec() * 1e6;

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = ros::Time::now();
    jerk_twists = chain->getDDTwist(q, Dq, DDq, DDDq);
    t_jerk_eigen += (ros::Time::now() - t0).toSec() * 1e6;

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = ros::Time::now();
    tau = chain->getJointTorque(q, Dq, DDq);
    t_torque_eigen += (ros::Time::now() - t0).toSec() * 1e6;

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = ros::Time::now();
    joint_inertia = chain->getJointInertia(q);
    t_inertia_eigen += (ros::Time::now() - t0).toSec() * 1e6;

    twist_of_tool_in_base = chain->getTwistTool(q, Dq);
    linacc_twist_of_tool_in_base = chain->getDTwistLinearPartTool(q, DDq);
    nonlinacc_twist_of_tool_in_base = chain->getDTwistNonLinearPartTool(q, Dq);
    acc_twist_of_tool_in_base = chain->getDTwistTool(q, Dq, DDq);
    jerk_twist_of_tool_in_base = chain->getDDTwistTool(q, Dq, DDq, DDDq);
  }

  ROS_INFO("average on %d trials: \nnote:\ncompute torque implies computing acceleration,\ncompute acceleration implies computing velocity,\ncompute velocity implies computing pose", ntrial);  // NOLINT(whitespace/line_length)
  ROS_INFO("computation time No operation                                    = %8.5f [us]", t_null / ntrial);
  ROS_INFO("computation time pose                                            = %8.5f [us]", t_pose_eigen / ntrial);
  ROS_INFO("computation time jacobian                                        = %8.5f [us]", t_jac_eigen / ntrial);
  ROS_INFO("computation time velocity twists for all links                   = %8.5f [us]", t_vel_eigen / ntrial);
  ROS_INFO("computation time linear raceleration twists for all links        = %8.5f [us]", t_linacc_eigen / ntrial);
  ROS_INFO("computation time non linear acceleration twists for all links    = %8.5f [us]", t_nonlinacc_eigen / ntrial);
  ROS_INFO("computation time acceleration twists for all links               = %8.5f [us]", t_acc_eigen  / ntrial);
  ROS_INFO("computation time jerk twists for all links                       = %8.5f [us]", t_jerk_eigen / ntrial);
  ROS_INFO("computation time joint torque                                    = %8.5f [us]", t_torque_eigen / ntrial);
  ROS_INFO("computation time joint inertia                                   = %8.5f [us]", t_inertia_eigen / ntrial);

  ros::Time st = ros::Time::now();
  std::thread::id this_id = std::this_thread::get_id();
  ros::Time et = ros::Time::now();
  ROS_INFO_STREAM("GET ID " << this_id << " | TIME  = " << (et-st).toSec() * 1e6 << "[us]");
  return 0;
}
