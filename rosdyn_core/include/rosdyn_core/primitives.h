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

#ifndef ROSDYN_CORE_PRIMITIVES_H
#define ROSDYN_CORE_PRIMITIVES_H

#include <string>
#include <vector>
#include <cmath>
#include <cstdint>
# include <assert.h>

# include <eigen3/Eigen/Geometry>
# include <eigen3/Eigen/StdVector>


# include <urdf/model.h>
# include <urdf_model/model.h>
# include <ros/console.h>

# include <rosdyn_core/urdf_parser.h>
# include <rosdyn_core/spacevect_algebra.h>
# include <rosdyn_core/ideal_spring.h>
# include <rosdyn_core/friction_polynomial2.h>
# include <rosdyn_core/friction_polynomial1.h>
# include <rosdyn_core/frame_distance.h>
# include <Eigen/Geometry>
# include<Eigen/StdVector>
# include <eigen_matrix_utils/eiquadprog.hpp>

#include <rosdyn_core/internal/types.h>

namespace rosdyn
{


class Joint: public shared_ptr_namespace::enable_shared_from_this<rosdyn::Joint>
{
public:
  enum Type {REVOLUTE, PRISMATIC, FIXED  };
protected:

  Type m_type;  // NOLINT(whitespace/braces)

  double m_q_max;
  double m_q_min;
  double m_Dq_max;
  double m_DDq_max;
  double m_tau_max;

  Eigen::Affine3d m_T_pj;           // transformation parent <- joint
  Eigen::Affine3d m_last_T_pc;      // transformation parent <- child
  Eigen::Matrix3d m_last_R_jc;      // rotatation     joint  <- child

  Eigen::Vector6d m_screw_of_c_in_p;  // skew of child origin in parent

  Eigen::Vector3d m_axis_in_j;
  Eigen::Matrix3d m_skew_axis_in_j;
  Eigen::Matrix3d m_square_skew_axis_in_j;
  Eigen::Matrix3d m_identity;

  Eigen::Vector3d m_axis_in_p;
  Eigen::Matrix3d m_skew_axis_in_p;
  Eigen::Matrix3d m_square_skew_axis_in_p;
  Eigen::Matrix3d m_R_pj;


  double m_last_q;  // last value of q

  std::string m_name;
  rosdyn::LinkPtr m_parent_link;
  rosdyn::LinkPtr m_child_link;

  void computedTpc();
  void computeJacobian();
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Joint();
  void fromUrdf(const urdf::JointPtr& urdf_joint, const rosdyn::LinkPtr& parent_link, const urdf::LinkPtr& child_link);
  int  enforceLimitsFromRobotDescriptionParam(const std::string& full_param_path, std::string& error);
  rosdyn::JointPtr pointer();
  const std::string& getName() const
  {
    return m_name;
  }

  rosdyn::LinkPtr getChildLink()
  {
    return m_child_link;
  }

  rosdyn::LinkPtr getParentLink()
  {
    return m_parent_link;
  }

  const Type& getType()const {return m_type;}

  const Eigen::Affine3d& getTransformation(const double& q = 0);
  const Eigen::Vector6d& getScrew_of_child_in_parent();

  const double& getQMax() const
  {
    return m_q_max;
  }
  const double& getQMin() const
  {
    return m_q_min;
  }
  const double& getDQMax() const
  {
    return m_Dq_max;
  }
  const double& getDDQMax() const
  {
    return m_DDq_max;
  }
  const double& getTauMax() const
  {
    return m_tau_max;
  }

  bool isFixed() const
  {
    return (m_type == FIXED);
  }
};

class Link: public shared_ptr_namespace::enable_shared_from_this<rosdyn::Link>
{
protected:
  rosdyn::JointPtr m_parent_joint;
  std::vector<rosdyn::JointPtr> m_child_joints;
  std::vector<rosdyn::LinkPtr> m_child_links;
  std::string m_name;

  double m_mass;
  Eigen::Vector3d m_cog_in_c;
  Eigen::Matrix<double, 6, 6> m_Inertia_cc;
  rosdyn::VectorOfMatrix66d m_Inertia_cc_single_term;


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Link() {}

  void fromUrdf(const urdf::LinkPtr& urdf_link,
                const rosdyn::JointPtr& parent_joint = 0);
  rosdyn::LinkPtr pointer();

  const std::string& getName() const
  {
    return m_name;
  }

  rosdyn::JointPtr getParentJoint()
  {
    return m_parent_joint;
  }

  std::vector<rosdyn::JointPtr> getChildrenJoints()
  {
    return m_child_joints;
  }

  rosdyn::LinkPtr findChild(const std::string& name);
  rosdyn::JointPtr findChildJoint(const std::string& name);
  const Eigen::Matrix66d& getSpatialInertia() const
  {
    return m_Inertia_cc;
  }

  const rosdyn::VectorOfMatrix66d& getSpatialInertiaTerms() const
  {
    return m_Inertia_cc_single_term;
  }

  const double& getMass() const
  {
    return m_mass;
  }

  const Eigen::Vector3d& getCog() const
  {
    return m_cog_in_c;
  }

  Eigen::VectorXd getNominalParameters() const;
};

class Chain
{
protected:
  std::vector<rosdyn::LinkPtr> m_links;
  std::vector<rosdyn::JointPtr> m_joints;
  unsigned int m_joints_number;
  unsigned int m_active_joints_number;
  unsigned int m_links_number;

  std::vector<std::string> m_links_name;
  std::map<std::string, unsigned int> m_joints_name;
  std::vector<std::string> m_moveable_joints_name;
  std::vector<std::string> m_active_joints_name;

  std::map<std::string, std::vector<unsigned int>> m_parent_moveable_joints_of_link;

  Eigen::Matrix6Xd m_jacobian;

  Eigen::Affine3d m_T_bt;                               // base <- tool

  Eigen::VectorXd m_last_q;
  Eigen::VectorXd m_sorted_q;

  Eigen::VectorXd m_last_Dq;
  Eigen::VectorXd m_sorted_Dq;

  Eigen::VectorXd m_last_DDq;
  Eigen::VectorXd m_sorted_DDq;

  Eigen::VectorXd m_last_DDDq;
  Eigen::VectorXd m_sorted_DDDq;

  Eigen::VectorXd m_q_max;
  Eigen::VectorXd m_q_min;
  Eigen::VectorXd m_Dq_max;
  Eigen::VectorXd m_DDq_max;
  Eigen::VectorXd m_tau_max;

  // for QP local ik solver
  Eigen::MatrixXd m_CE;
  Eigen::VectorXd m_ce0;
  Eigen::MatrixXd m_CI;
  Eigen::VectorXd m_ci0;
  Eigen::MatrixXd m_H;
  Eigen::VectorXd m_f;
  Eigen::VectorXd m_joint_error;
  Eigen::Vector6d m_cart_error_in_b;


  Eigen::VectorXd m_joint_torques;
  Eigen::VectorXd m_active_joint_torques;
  Eigen::MatrixXd m_regressor_extended;

  Eigen::MatrixXd m_input_to_chain_joint;
  Eigen::MatrixXd m_chain_to_input_joint;
  std::vector<unsigned int> m_active_joints;

  rosdyn::VectorOfAffine3d m_T_bl;
  bool m_is_screws_computed;
  bool m_is_jac_computed;
  bool m_is_vel_computed;
  bool m_is_acc_computed;
  bool m_is_linacc_computed;
  bool m_is_nonlinacc_computed;
  bool m_is_jerk_computed;
  bool m_is_linjerk_computed;
  bool m_is_nonlinjerk_computed;
  bool m_is_wrench_computed;
  bool m_is_regressor_computed;
  bool m_is_chain_ok = true;

  rosdyn::VectorOfVector6d m_screws_of_c_in_b;

  rosdyn::VectorOfVector6d m_Ds;

  // twists of c in b
  rosdyn::VectorOfVector6d m_twists;
  // Dtwists of c in b
  rosdyn::VectorOfVector6d m_Dtwists;
  rosdyn::VectorOfVector6d m_Dtwists_linear_part;
  rosdyn::VectorOfVector6d m_Dtwists_nonlinear_part;

  rosdyn::VectorOfVector6d m_DDtwists;
  rosdyn::VectorOfVector6d m_DDtwists_linear_part;
  rosdyn::VectorOfVector6d m_DDtwists_nonlinear_part;

  rosdyn::VectorOfVector6d m_wrenches;
  rosdyn::VectorOfVector6d m_inertial_wrenches;
  rosdyn::VectorOfVector6d m_gravity_wrenches;
  rosdyn::VectorOfMatrix610d m_wrenches_regressor;

  Eigen::Vector3d m_gravity;
  Eigen::MatrixXd m_joint_inertia;
  Eigen::MatrixXd m_joint_inertia_extended;

  void computeScrews();
  void computeFrames();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Chain() = default;
  ~Chain() = default;
  Chain(const Chain&);
  Chain(Chain&&) = delete;
  Chain& operator=(const Chain&);
  Chain& operator=(Chain&&) = delete;

  Chain(const rosdyn::LinkPtr& root_link,
          const std::string& base_link_name,
            const std::string& ee_link_name,
              const Eigen::Vector3d& gravity = Eigen::Vector3d::Zero());
  Chain(const urdf::Model& model,
          const std::string& base_link_name,
            const std::string& ee_link_name, 
              const Eigen::Vector3d& gravity = Eigen::Vector3d::Zero());
  Chain(const std::string& robot_description,
          const std::string& base_link_name,
            const std::string& ee_link_name,
              const Eigen::Vector3d& gravity = Eigen::Vector3d::Zero());
  bool init(std::string& error,
            rosdyn::LinkPtr root_link,
              const std::string& base_link_name,
                const std::string& ee_link_name,
                  const Eigen::Vector3d& gravity = Eigen::Vector3d::Zero());

  // true: all the joints are in the chain descriptor (from urdf), false: at least one joint is not listed
  bool setInputJointsName(const std::vector<std::string>& joints_name);
  int  enforceLimitsFromRobotDescriptionParam(const std::string& full_param_path, std::string& error);
  const std::vector<std::string>& getMoveableJointNames() const
  {
    return m_moveable_joints_name;
  }
  const std::string& getMoveableJointName(const size_t& iAx) const
  {
    return getMoveableJointNames().at(iAx);
  }
  const unsigned int& getLinksNumber() const
  {
    return m_links_number;
  }
  const unsigned int& getJointsNumber() const
  {
    return m_joints_number;
  }
  const unsigned int& getActiveJointsNumber() const
  {
    return m_active_joints_number;
  }
  const std::vector<std::string>& getActiveJointsName() const
  {
    return m_active_joints_name;
  }
  const std::string& getActiveJointName(const size_t& iAx) const
  {
    return m_active_joints_name.at(iAx);
  }
  const std::vector<std::string>& getLinksName() const
  {
    return m_links_name;
  }
  const std::vector<rosdyn::LinkPtr>& getLinks() const
  {
    return m_links;
  }
  const std::vector<rosdyn::JointPtr>& getJoints() const
  {
    return m_joints;
  }
  const Eigen::Vector3d& getGravity() const
  {
    return m_gravity;
  }
  const bool& isOk() const
  {
    return m_is_chain_ok;
  }
  const Eigen::VectorXd& getQMax() const
  {
    return m_q_max;
  }
  const Eigen::VectorXd& getQMin() const
  {
    return m_q_min;
  }
  const Eigen::VectorXd& getDQMax() const
  {
    return m_Dq_max;
  }
  const Eigen::VectorXd& getDDQMax() const
  {
    return m_DDq_max;
  }
  const Eigen::VectorXd& getTauMax() const
  {
    return m_tau_max;
  }
  const double& getQMax   (int iAx) const { return m_q_max(iAx); }
  const double& getQMin   (int iAx) const { return m_q_min(iAx); }
  const double& getDQMax  (int iAx) const { return m_Dq_max(iAx);    }
  const double& getDDQMax (int iAx) const { return m_DDq_max(iAx);   }
  const double& getTauMax (int iAx) const { return m_tau_max(iAx);   }

  double getQMax   (const std::string& name) const {int idx=jointIndex(name); return idx==-1 ? NAN : m_q_max(idx); }
  double getQMin   (const std::string& name) const {int idx=jointIndex(name); return idx==-1 ? NAN : m_q_min(idx); }
  double getDQMax  (const std::string& name) const {int idx=jointIndex(name); return idx==-1 ? NAN : m_Dq_max(idx); }
  double getDDQMax (const std::string& name) const {int idx=jointIndex(name); return idx==-1 ? NAN : m_DDq_max(idx); }
  double getTauMax (const std::string& name) const {int idx=jointIndex(name); return idx==-1 ? NAN : m_tau_max(idx); }
  int jointIndex (const std::string& name) const
  {
    auto it = std::find(m_moveable_joints_name.begin(),m_moveable_joints_name.end(), name);
    return it == m_moveable_joints_name.end() ? -1 : std::distance(m_moveable_joints_name.begin(), it);
  }

  /*
   * Kinematics methods
   */
  const Eigen::Affine3d& getTransformation(const Eigen::VectorXd& q);
  const Eigen::Affine3d& getTransformationLink(const Eigen::VectorXd& q, const std::string& link_name);
  const rosdyn::VectorOfAffine3d& getTransformations(const Eigen::VectorXd& q);
  const Eigen::Matrix6Xd& getJacobian(const Eigen::VectorXd& q);
  Eigen::Matrix6Xd getJacobianLink(const Eigen::VectorXd& q,const std::string& link_name);
  const rosdyn::VectorOfVector6d& getTwist(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq);
  const Eigen::Vector6d& getTwistLink(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const std::string& link_name);
  const Eigen::Vector6d& getTwistTool(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq)
  {
    return getTwist(q, Dq).back();
  }
  const rosdyn::VectorOfVector6d& getDTwist(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq);
  const Eigen::Vector6d& getDTwistTool(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq)
  {
    return getDTwist(q, Dq, DDq).back();
  }
  const rosdyn::VectorOfVector6d& getDTwistLinearPart(const Eigen::VectorXd& q, const Eigen::VectorXd& DDq);
  const Eigen::Vector6d& getDTwistLinearPartTool(const Eigen::VectorXd& q, const Eigen::VectorXd& DDq)
  {
    return getDTwistLinearPart(q, DDq).back();
  }
  const rosdyn::VectorOfVector6d& getDTwistNonLinearPart(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq);
  const Eigen::Vector6d& getDTwistNonLinearPartTool(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq)
  {
    return getDTwistNonLinearPart(q, Dq).back();
  }
  const rosdyn::VectorOfVector6d& getDDTwistLinearPart(const Eigen::VectorXd& q, const Eigen::VectorXd& DDDq);
  const Eigen::Vector6d& getDDTwistLinearPartTool(const Eigen::VectorXd& q, const Eigen::VectorXd& DDDq)
  {
    return getDDTwistLinearPart(q, DDDq).back();
  }
  const rosdyn::VectorOfVector6d& getDDTwistNonLinearPart(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq);
  const Eigen::Vector6d& getDDTwistNonLinearPartTool(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq)
  {
    return getDDTwistNonLinearPart(q, Dq, DDq).back();
  }
  const rosdyn::VectorOfVector6d& getDDTwist(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq, const Eigen::VectorXd& DDDq);
  const Eigen::Vector6d& getDDTwistTool(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq, const Eigen::VectorXd& DDDq)
  {
    return getDDTwist(q, Dq, DDq, DDDq).back();
  }

  /* get all the multiturn solution. q is in the output vector.
   */
  std::vector<Eigen::VectorXd> getMultiplicity(const Eigen::VectorXd& q);

  /* Iterative solve the IK problem.
   * Each iteration the solution is updated by a term dq
   *
   * solution+=dq
   *
   * the value dq is chosen solving the problem:
   * minimize (J*dq-cartesian_error_in_b)'*(J*dq-cartesian_error_in_b)
   *
   * considering the constraints:
   * q_min <= solution+dq <= q_max
   *
   */
  bool computeLocalIk(Eigen::VectorXd& sol, const Eigen::Affine3d& T_b_t, const Eigen::VectorXd& seed, const double& toll = 1e-4, const ros::Duration& max_time = ros::Duration(0.005));

  /* Iterative solve the weighted IK problem.
   * the weight term can be used to switch off some components:
   * for example: selecting W=[1;1;1;0;0;0], only the translation terms are consider for the IK solution.
   * Each iteration the solution is updated by a term dq
   *
   * solution+=dq
   *
   * the value dq is chosen solving the problem:
   * minimize (J*dq-cartesian_error_in_b)'*W*(J*dq-cartesian_error_in_b)
   *
   * considering the constraints:
   * q_min <= solution+dq <= q_max
   *
   */
  bool computeWeigthedLocalIk(Eigen::VectorXd& sol, const Eigen::Affine3d& T_b_t, Eigen::Vector6d weight, const Eigen::VectorXd& seed, const double& toll = 1e-4, const ros::Duration& max_time = ros::Duration(0.005));

  /*
   * Dynamics methods
   */
  const rosdyn::VectorOfVector6d& getWrench(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq, rosdyn::VectorOfVector6d& ext_wrenches_in_link_frame);
  const Eigen::Vector6d& getWrenchTool(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq, rosdyn::VectorOfVector6d& ext_wrenches_in_link_frame)
  {
    return getWrench(q, DDq, DDq, ext_wrenches_in_link_frame).back();
  }
  const Eigen::VectorXd& getJointTorque(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq, rosdyn::VectorOfVector6d& ext_wrenches_in_link_frame);
  const Eigen::VectorXd& getJointTorque(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq);
  const Eigen::VectorXd& getJointTorqueNonLinearPart(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq);

  Eigen::MatrixXd getRegressor(const Eigen::VectorXd& q,
                               const Eigen::VectorXd& Dq,
                               const Eigen::VectorXd& DDq);

  const Eigen::MatrixXd& getJointInertia(const Eigen::VectorXd& q);
  Eigen::VectorXd getNominalParameters();

};


/**
 * @brief construct the shared_ptr of a chain
 * 
 * @param[in] urdf
 * @param[in] base frame
 * @param[in] tool frame
 * @param[in] gravity vector
 */
rosdyn::ChainPtr createChain( const urdf::ModelInterface& urdf_model_interface,
                              const std::string& base_frame,
                              const std::string& tool_frame, 
                              const Eigen::Vector3d& gravity);


/**
 * @brief construct the shared_ptr of a chain
 * 
 * @param[in] Chain
 */
rosdyn::ChainPtr createChain(const rosdyn::ChainPtr& chain);


///////////////////////////////////////////////////

}  // namespace rosdyn

#include <rosdyn_core/internal/primitives_impl.h>

#endif  // ROSDYN_CORE_PRIMITIVES_H 
