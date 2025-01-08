#include "rdyn_core/primitives.h"
#include "rdyn_core/internal/primitives_impl.h"
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <rdyn_core/urdf_parser.h>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#if !defined(PROJECT_SRC_DIRECTORY)
  #error "The test need that the src directory is pre-compiled. Check the CMAKE";
#else
  constexpr const char* _PROJECT_SRC_DIRECTORY = PROJECT_SRC_DIRECTORY;
#endif

namespace {

class RdynCoreTest : public ::testing::Test {
protected:
  void SetUp() override {
    src_path = boost::filesystem::path(_PROJECT_SRC_DIRECTORY);
    urdf_path = src_path / "ur10.urdf";
    base_frame = "base_link";
    tool_frame = "tool0";
    grav << 0, 0, -9.806;
    chain = rdyn::createChainFromFile(urdf_path.string(), base_frame, tool_frame, grav);
    n_joints = chain->getActiveJointsNumber();
  }

  boost::filesystem::path src_path;
  boost::filesystem::path urdf_path;
  std::string base_frame;
  std::string tool_frame;
  Eigen::Vector3d grav;
  shared_ptr_namespace::shared_ptr<rdyn::Chain> chain;
  unsigned int n_joints;
};

TEST_F(RdynCoreTest, ChainCreation) {
  ASSERT_NE(chain, nullptr);
  ASSERT_EQ(chain->getActiveJointsNumber(), n_joints);
}

TEST_F(RdynCoreTest, PoseComputation) {
  Eigen::VectorXd q(n_joints);
  q.setRandom();
  Eigen::Affine3d T_base_tool = chain->getTransformation(q);
  ASSERT_TRUE(T_base_tool.matrix().allFinite());
}

TEST_F(RdynCoreTest, JacobianComputation) {
  Eigen::VectorXd q(n_joints);
  q.setRandom();
  Eigen::Matrix6Xd jacobian_of_tool_in_base;
  jacobian_of_tool_in_base.resize(6, n_joints);
  jacobian_of_tool_in_base = chain->getJacobian(q);
  ASSERT_TRUE(jacobian_of_tool_in_base.allFinite());
}

TEST_F(RdynCoreTest, TwistComputation) {
  Eigen::VectorXd q(n_joints);
  Eigen::VectorXd Dq(n_joints);
  q.setRandom();
  Dq.setRandom();
  std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> twists;
  twists = chain->getTwist(q, Dq);
  for (const auto& twist : twists) {
    ASSERT_TRUE(twist.allFinite());
  }
}

TEST_F(RdynCoreTest, LinearAccelerationTwistComputation) {
  Eigen::VectorXd q(n_joints);
  Eigen::VectorXd DDq(n_joints);
  q.setRandom();
  DDq.setRandom();
  std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> linacc_twists;
  linacc_twists = chain->getDTwistLinearPart(q, DDq);
  for (const auto& linacc_twist : linacc_twists) {
    ASSERT_TRUE(linacc_twist.allFinite());
  }
}

TEST_F(RdynCoreTest, NonLinearAccelerationTwistComputation) {
  Eigen::VectorXd q(n_joints);
  Eigen::VectorXd Dq(n_joints);
  q.setRandom();
  Dq.setRandom();
  std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> nonlinacc_twists;
  nonlinacc_twists = chain->getDTwistNonLinearPart(q, Dq);
  for (const auto& nonlinacc_twist : nonlinacc_twists) {
    ASSERT_TRUE(nonlinacc_twist.allFinite());
  }
}

TEST_F(RdynCoreTest, AccelerationTwistComputation) {
  Eigen::VectorXd q(n_joints);
  Eigen::VectorXd Dq(n_joints);
  Eigen::VectorXd DDq(n_joints);
  q.setRandom();
  Dq.setRandom();
  DDq.setRandom();
  std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> acc_twists;
  acc_twists = chain->getDTwist(q, Dq, DDq);
  for (const auto& acc_twist : acc_twists) {
    ASSERT_TRUE(acc_twist.allFinite());
  }
}

TEST_F(RdynCoreTest, JerkTwistComputation) {
  Eigen::VectorXd q(n_joints);
  Eigen::VectorXd Dq(n_joints);
  Eigen::VectorXd DDq(n_joints);
  Eigen::VectorXd DDDq(n_joints);
  q.setRandom();
  Dq.setRandom();
  DDq.setRandom();
  DDDq.setRandom();
  std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> jerk_twists;
  jerk_twists = chain->getDDTwist(q, Dq, DDq, DDDq);
  for (const auto& jerk_twist : jerk_twists) {
    ASSERT_TRUE(jerk_twist.allFinite());
  }
}

TEST_F(RdynCoreTest, JointTorqueComputation) {
  Eigen::VectorXd q(n_joints);
  Eigen::VectorXd Dq(n_joints);
  Eigen::VectorXd DDq(n_joints);
  q.setRandom();
  Dq.setRandom();
  DDq.setRandom();
  Eigen::VectorXd tau = chain->getJointTorque(q, Dq, DDq);
  ASSERT_TRUE(tau.allFinite());
}

TEST_F(RdynCoreTest, JointInertiaComputation) {
  Eigen::VectorXd q(n_joints);
  q.setRandom();
  Eigen::MatrixXd joint_inertia = chain->getJointInertia(q);
  ASSERT_TRUE(joint_inertia.allFinite());
}

TEST_F(RdynCoreTest, ComputationTime) {
  Eigen::VectorXd q(n_joints);
  Eigen::VectorXd Dq(n_joints);
  Eigen::VectorXd DDq(n_joints);
  Eigen::VectorXd DDDq(n_joints);

  double t_pose_eigen = 0;
  double t_jac_eigen = 0;
  double t_vel_eigen = 0;
  double t_linacc_eigen = 0;
  double t_nonlinacc_eigen = 0;
  double t_acc_eigen = 0;
  double t_jerk_eigen = 0;
  double t_torque_eigen = 0;
  double t_inertia_eigen = 0;
  int ntrial = 1e6;

  for (int idx = 0; idx < ntrial; idx++) {
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();

    auto t0 = std::chrono::high_resolution_clock::now();
    chain->getTransformation(q);
    t_pose_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t0).count();

    t0 = std::chrono::high_resolution_clock::now();
    chain->getJacobian(q);
    t_jac_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t0).count();

    t0 = std::chrono::high_resolution_clock::now();
    chain->getTwist(q, Dq);
    t_vel_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t0).count();

    t0 = std::chrono::high_resolution_clock::now();
    chain->getDTwistLinearPart(q, DDq);
    t_linacc_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t0).count();

    t0 = std::chrono::high_resolution_clock::now();
    chain->getDTwistNonLinearPart(q, Dq);
    t_nonlinacc_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t0).count();

    t0 = std::chrono::high_resolution_clock::now();
    chain->getDTwist(q, Dq, DDq);
    t_acc_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t0).count();

    t0 = std::chrono::high_resolution_clock::now();
    chain->getDDTwist(q, Dq, DDq, DDDq);
    t_jerk_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t0).count();

    t0 = std::chrono::high_resolution_clock::now();
    chain->getJointTorque(q, Dq, DDq);
    t_torque_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t0).count();

    t0 = std::chrono::high_resolution_clock::now();
    chain->getJointInertia(q);
    t_inertia_eigen += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t0).count();
  }

  printf("Average on %d trials:\n", ntrial);
  printf("Computation time pose                                            = %8.5f [us]\n", t_pose_eigen / ntrial);
  printf("Computation time jacobian                                        = %8.5f [us]\n", t_jac_eigen / ntrial);
  printf("Computation time velocity twists for all links                   = %8.5f [us]\n", t_vel_eigen / ntrial);
  printf("Computation time linear acceleration twists for all links        = %8.5f [us]\n", t_linacc_eigen / ntrial);
  printf("Computation time non linear acceleration twists for all links    = %8.5f [us]\n", t_nonlinacc_eigen / ntrial);
  printf("Computation time acceleration twists for all links               = %8.5f [us]\n", t_acc_eigen / ntrial);
  printf("Computation time jerk twists for all links                       = %8.5f [us]\n", t_jerk_eigen / ntrial);
  printf("Computation time joint torque                                    = %8.5f [us]\n", t_torque_eigen / ntrial);
  printf("Computation time joint inertia                                   = %8.5f [us]\n", t_inertia_eigen / ntrial);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
