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

#include <chrono>
#include <boost/filesystem.hpp>

#include <rdyn_core/primitives.h>
#include <rdyn_core/internal/types.h>
#include <rdyn_core/kinematics_saturation.h>
#include <rdyn_core/urdf_parser.h>
#include <string>
#include <vector>
#include <gtest/gtest.h>

#include <thread>

rdyn::ChainPtr chain;


#if !defined(PROJECT_SRC_DIRECTORY)
  #error "The test need that the src directory is pre-compiled. Check the CMAKE";
#else
  constexpr const char* _PROJECT_SRC_DIRECTORY = PROJECT_SRC_DIRECTORY;
#endif

TEST(Suite, chainPtrTest)
{
  boost::filesystem::path src_path(_PROJECT_SRC_DIRECTORY);
  boost::filesystem::path urdf_path = src_path / "test/ur10.urdf";
  std::cout << "The config file path is : " << urdf_path << std::endl;	

  std::string base_frame = "base_link";
  std::string tool_frame = "tool0";

  urdf::Model model;
  model.initFile(urdf_path.string());
  
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  chain = rdyn::createChain(model, base_frame, tool_frame, grav);

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

  rdyn::VectorOfVector6d twists, nonlinacc_twists, linacc_twists, acc_twists, jerk_twists;  // NOLINT(whitespace/line_length)

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
  auto t0 = std::chrono::steady_clock::now();
  int ntrial = 1e3;


  for (int idx = 0; idx < ntrial; idx++)
  {
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();

    t0 = std::chrono::steady_clock::now();
    t_null += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();
    t0 = std::chrono::steady_clock::now();

    T_base_tool = chain->getTransformation(q);
    t_pose_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = std::chrono::steady_clock::now();
    jacobian_of_tool_in_base = chain->getJacobian(q);
    t_jac_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = std::chrono::steady_clock::now();
    twists = chain->getTwist(q, Dq);
    t_vel_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = std::chrono::steady_clock::now();
    linacc_twists = chain->getDTwistLinearPart(q, DDq);
    t_linacc_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = std::chrono::steady_clock::now();
    nonlinacc_twists = chain->getDTwistNonLinearPart(q, Dq);
    t_nonlinacc_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = std::chrono::steady_clock::now();
    acc_twists = chain->getDTwist(q, Dq, DDq);
    t_acc_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = std::chrono::steady_clock::now();
    jerk_twists = chain->getDDTwist(q, Dq, DDq, DDDq);
    t_jerk_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = std::chrono::steady_clock::now();
    tau = chain->getJointTorque(q, Dq, DDq);
    t_torque_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = std::chrono::steady_clock::now();
    joint_inertia = chain->getJointInertia(q);
    t_inertia_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    twist_of_tool_in_base = chain->getTwistTool(q, Dq);
    linacc_twist_of_tool_in_base = chain->getDTwistLinearPartTool(q, DDq);
    nonlinacc_twist_of_tool_in_base = chain->getDTwistNonLinearPartTool(q, Dq);
    acc_twist_of_tool_in_base = chain->getDTwistTool(q, Dq, DDq);
    jerk_twist_of_tool_in_base = chain->getDDTwistTool(q, Dq, DDq, DDDq);
  }

  printf("average on %d trials: \nnote:\ncompute torque implies computing acceleration,\ncompute acceleration implies computing velocity,\ncompute velocity implies computing pose\n", ntrial);  // NOLINT(whitespace/line_length)
  printf("computation time No operation                                    = %8.5f [us]\n", t_null / ntrial);
  printf("computation time pose                                            = %8.5f [us]\n", t_pose_eigen / ntrial);
  printf("computation time jacobian                                        = %8.5f [us]\n", t_jac_eigen / ntrial);
  printf("computation time velocity twists for all links                   = %8.5f [us]\n", t_vel_eigen / ntrial);
  printf("computation time linear raceleration twists for all links        = %8.5f [us]\n", t_linacc_eigen / ntrial);
  printf("computation time non linear acceleration twists for all links    = %8.5f [us]\n", t_nonlinacc_eigen / ntrial);
  printf("computation time acceleration twists for all links               = %8.5f [us]\n", t_acc_eigen  / ntrial);
  printf("computation time jerk twists for all links                       = %8.5f [us]\n", t_jerk_eigen / ntrial);
  printf("computation time joint torque                                    = %8.5f [us]\n", t_torque_eigen / ntrial);
  printf("computation time joint inertia                                   = %8.5f [us]\n", t_inertia_eigen / ntrial);

  DELETE_HEAP(chain);
}


TEST(Suite, staticChainTest)
{
  boost::filesystem::path src_path(_PROJECT_SRC_DIRECTORY);
  boost::filesystem::path urdf_path = src_path / "test/ur10.urdf";
  std::cout << "The config file path is : " << urdf_path << std::endl;	

  std::string base_frame = "base_link";
  std::string tool_frame = "tool0";

  urdf::Model model;
  EXPECT_TRUE(model.initFile(urdf_path.string()) );

  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  rdyn::Chain chain;
  std::string error;
  rdyn::LinkPtr root_link;
  NEW_HEAP(root_link, rdyn::Link());
  root_link->fromUrdf(GET(model.root_link_));
  chain.init(error,root_link, base_frame, tool_frame, grav);

  unsigned int n_joints = chain.getActiveJointsNumber();
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

  rdyn::VectorOfVector6d twists, nonlinacc_twists, linacc_twists, acc_twists, jerk_twists;  // NOLINT(whitespace/line_length)

  Eigen::Matrix6Xd jacobian_of_tool_in_base;
  jacobian_of_tool_in_base.resize(6, chain.getActiveJointsNumber());
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
  auto t0 = std::chrono::steady_clock::now();
  int ntrial = 1e3;


  for (int idx = 0; idx < ntrial; idx++)
  {
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();

    t0 = std::chrono::steady_clock::now();
    t_null += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();
    t0 = std::chrono::steady_clock::now();

    T_base_tool = chain.getTransformation(q);
    t_pose_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = std::chrono::steady_clock::now();
    jacobian_of_tool_in_base = chain.getJacobian(q);
    t_jac_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = std::chrono::steady_clock::now();
    twists = chain.getTwist(q, Dq);
    t_vel_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = std::chrono::steady_clock::now();
    linacc_twists = chain.getDTwistLinearPart(q, DDq);
    t_linacc_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = std::chrono::steady_clock::now();
    nonlinacc_twists = chain.getDTwistNonLinearPart(q, Dq);
    t_nonlinacc_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = std::chrono::steady_clock::now();
    acc_twists = chain.getDTwist(q, Dq, DDq);
    t_acc_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = std::chrono::steady_clock::now();
    jerk_twists = chain.getDDTwist(q, Dq, DDq, DDDq);
    t_jerk_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = std::chrono::steady_clock::now();
    tau = chain.getJointTorque(q, Dq, DDq);
    t_torque_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = std::chrono::steady_clock::now();
    joint_inertia = chain.getJointInertia(q);
    t_inertia_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();

    twist_of_tool_in_base = chain.getTwistTool(q, Dq);
    linacc_twist_of_tool_in_base = chain.getDTwistLinearPartTool(q, DDq);
    nonlinacc_twist_of_tool_in_base = chain.getDTwistNonLinearPartTool(q, Dq);
    acc_twist_of_tool_in_base = chain.getDTwistTool(q, Dq, DDq);
    jerk_twist_of_tool_in_base = chain.getDDTwistTool(q, Dq, DDq, DDDq);
  }

  printf("average on %d trials: \nnote:\ncompute torque implies computing acceleration,\ncompute acceleration implies computing velocity,\ncompute velocity implies computing pose\n", ntrial);  // NOLINT(whitespace/line_length)
  printf("computation time No operation                                    = %8.5f [us]\n", t_null / ntrial);
  printf("computation time pose                                            = %8.5f [us]\n", t_pose_eigen / ntrial);
  printf("computation time jacobian                                        = %8.5f [us]\n", t_jac_eigen / ntrial);
  printf("computation time velocity twists for all links                   = %8.5f [us]\n", t_vel_eigen / ntrial);
  printf("computation time linear raceleration twists for all links        = %8.5f [us]\n", t_linacc_eigen / ntrial);
  printf("computation time non linear acceleration twists for all links    = %8.5f [us]\n", t_nonlinacc_eigen / ntrial);
  printf("computation time acceleration twists for all links               = %8.5f [us]\n", t_acc_eigen  / ntrial);
  printf("computation time jerk twists for all links                       = %8.5f [us]\n", t_jerk_eigen / ntrial);
  printf("computation time joint torque                                    = %8.5f [us]\n", t_torque_eigen / ntrial);
  printf("computation time joint inertia                                   = %8.5f [us]\n", t_inertia_eigen / ntrial);
}


TEST(Suite, threadID)
{

    auto t0 = std::chrono::steady_clock::now();

    t0 = std::chrono::steady_clock::now();
    std::thread::id this_id = std::this_thread::get_id();
    auto t1 = std::chrono::steady_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    std::cout << "GET ID " << this_id << " | TIME  = " << dt << "[us]" << std::endl;
    int ntrial = 1e4;
    double t_null = 0;
    double t_max = 0;

    std::vector<std::thread> grp;
    for (int idx = 0; idx < ntrial; idx++)
    {
        grp.emplace_back([&t_null, &t_max]()
        {
            auto t0 = std::chrono::steady_clock::now();
            std::thread::id this_id = std::this_thread::get_id();
            auto t1 = std::chrono::steady_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
            t_null += dt;
            t_max = std::max(t_max, dt);
        });
    }
    for (auto& thread : grp)
        thread.join();

    printf("GET ID MEAN TIME = %8.5f [us]\n", t_null / 1000.0 / ntrial);
    printf("GET ID MAX TIME = %8.5f [us]\n", t_max / 1000.0);
}



int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  
  return RUN_ALL_TESTS();

}
