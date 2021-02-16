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

#ifndef ROSDYM_CORE__PRIMITIVES_H
#define ROSDYM_CORE__PRIMITIVES_H

#include <string>
#include <vector>
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
# include <Eigen/StdVector>
# include <eigen_matrix_utils/eiquadprog.hpp>

#if ROS_VERSION_MINIMUM(1, 14, 1)
# include <memory>
namespace shared_ptr_namespace = std;
#else
# include <boost/concept_check.hpp>
# include <boost/graph/graph_concepts.hpp>
# include <boost/enable_shared_from_this.hpp>
namespace shared_ptr_namespace = boost;
#endif

namespace urdf
{
  typedef shared_ptr_namespace::shared_ptr< urdf::Joint      > JointPtr;
  typedef shared_ptr_namespace::shared_ptr< urdf::Link       > LinkPtr;
  typedef shared_ptr_namespace::shared_ptr< urdf::Joint const> JointConstPtr;
  typedef shared_ptr_namespace::shared_ptr< urdf::Link  const> LinkConstPtr;
}

namespace rosdyn
{
class Joint;
class Link;
class Chain;

typedef shared_ptr_namespace::shared_ptr< rosdyn::Joint   > JointPtr;
typedef shared_ptr_namespace::shared_ptr< rosdyn::Link    > LinkPtr;
typedef shared_ptr_namespace::shared_ptr< rosdyn::Chain   > ChainPtr;

typedef shared_ptr_namespace::shared_ptr< rosdyn::Joint const> JointConstPtr;
typedef shared_ptr_namespace::shared_ptr< rosdyn::Link  const> LinkConstPtr;
typedef shared_ptr_namespace::shared_ptr< rosdyn::Chain const> ChainConstPtr;

#if defined(MAX_NUM_AXES) && (MAX_NUM_AXES!=0)
  #define STR_HELPER(x) #x
  #define STR(x) STR_HELPER(x)
  #pragma message "ROSDYN MAX NUM AXES: " STR(MAX_NUM_AXES)
  #define NUM_MAX_AXES MAX_NUM_AXES
#else
  #define NUM_MAX_AXES 40
#endif

constexpr int max_num_axes = NUM_MAX_AXES;
constexpr int max_num_extended_axes = (NUM_MAX_AXES > 0 ? NUM_MAX_AXES * 10 : -1);
typedef Eigen::Matrix<double,-1, 1,Eigen::ColMajor,max_num_axes> VectorXd;
typedef Eigen::Matrix<double,-1,-1,Eigen::ColMajor,max_num_axes,max_num_axes> MatrixXd;
typedef Eigen::Matrix<double, 6,-1,Eigen::ColMajor,           6,max_num_axes> Matrix6Xd;
typedef Eigen::Matrix<double, 6, 6>                                           Matrix66d;
typedef Eigen::Matrix<double,-1,-1,Eigen::ColMajor,max_num_axes,max_num_extended_axes> ExtendedMatrixXd;
typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> VectorOfAffine3d;
typedef std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> VectorOfVector6d;
typedef std::vector< Matrix66d, Eigen::aligned_allocator< Matrix66d > >         VectorOfMatrix66d;
typedef Eigen::Matrix<double, 6, 10>                                            Matrix610d;
typedef std::vector< Matrix610d, Eigen::aligned_allocator< Matrix610d > >       VectorOfMatrix610d;

class Joint //: public shared_ptr_namespace::enable_shared_from_this<rosdyn::Joint>
{
protected:

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

  double m_q_max;
  double m_q_min;
  double m_Dq_max;
  double m_DDq_max;
  double m_tau_max;
  double m_last_q;  // last value of q

  std::string m_name;
  rosdyn::Link* m_parent_link;
  rosdyn::Link* m_child_link;

  enum
  {
    REVOLUTE, PRISMATIC, FIXED
  } m_type;  // NOLINT(whitespace/braces)

  void computedTpc();
  void computeJacobian();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Joint();
  ~Joint() = default;

  void fromUrdf(const urdf::Joint* urdf_joint, Link* parent_link, const urdf::Link* child_link);
  int  enforceLimitsFromRobotDescriptionParam(const std::string& full_param_path, std::string& error);
  rosdyn::Joint* pointer();
  const rosdyn::Joint* pointer() const;
  const std::string& getName() const
  {
    return m_name;
  }

  rosdyn::Link* getChildLink()
  {
    return m_child_link;
  }

  const rosdyn::Link* getChildLink() const
  {
    return m_child_link;
  }

  rosdyn::Link* getParentLink()
  {
    return m_parent_link;
  }

  const rosdyn::Link*  getParentLink() const
  {
    return m_parent_link;
  }


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

/**
 * @brief The Link class
 */
class Link // : public shared_ptr_namespace::enable_shared_from_this<rosdyn::Link>
{
private:
  Eigen::Vector3d m_cog_in_c;
  rosdyn::Matrix66d m_Inertia_cc;
  rosdyn::VectorOfMatrix66d m_Inertia_cc_single_term;

  rosdyn::Joint* m_parent_joint;

  std::vector<rosdyn::Joint* > m_child_joints;
  std::vector<rosdyn::Link* > m_child_links;
  std::string m_name;

  double m_mass;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Link() = default;
  ~Link() = default;

  void fromUrdf(const urdf::Link* urdf_link, rosdyn::Joint* parent_joint = nullptr);
  rosdyn::Link* pointer();
  const rosdyn::Link* pointer() const;

  std::string getName() const
  {
    return m_name;
  }

  rosdyn::Joint* getParentJoint()
  {
    return m_parent_joint;
  }

  const rosdyn::Joint* getParentJoint() const
  {
    return m_parent_joint;
  }

  const std::vector<rosdyn::Joint*>& getChildrenJoints() const
  {
    return m_child_joints;
  }

  rosdyn::Link* findChild(const std::string& name);
  rosdyn::Joint* findChildJoint(const std::string& name);

  const rosdyn::Link*  findChild(const std::string& name) const;
  const rosdyn::Joint* findChildJoint(const std::string& name) const;

  const Eigen::Matrix66d& getSpatialInertia()
  {
    return m_Inertia_cc;
  }

  const VectorOfMatrix66d& getSpatialInertiaTerms()
  {
    return m_Inertia_cc_single_term;
  }

  const double& getMass()
  {
    return m_mass;
  }

  const Eigen::Vector3d& getCog()
  {
    return m_cog_in_c;
  }

  Eigen::VectorXd getNominalParameters();
};

/**
 * @brief The Chain class
 */
class Chain
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Chain() = default;
  ~Chain() = default;
  Chain(const Chain&) = delete;
  Chain(Chain&&) = delete;
  Chain& operator=(const Chain&) = delete;
  Chain& operator=(Chain&&) = delete;

  Chain(rosdyn::Link* root_link,
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
              rosdyn::Link* root_link,
                const std::string& base_link_name,
                  const std::string& ee_link_name,
                    const Eigen::Vector3d& gravity = Eigen::Vector3d::Zero());

  /**
   * @brief setInputJointsName
   * @param joints_name
   * @return 1 ok, -1 error, 0 warnings (the joint_names are not in the list of the URDF)
   */
  int setInputJointsName(const std::vector<std::string> joints_name);
  int  enforceLimitsFromRobotDescriptionParam(const std::string& full_param_path, std::string& error);

  const std::vector<std::string>& getMoveableJointNames() const
  {
    return m_moveable_joints_name;
  }
  const std::string& getJointName(const size_t iAx) const { return getMoveableJointNames().at(iAx);}

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

  const std::vector<std::string>& getLinksName() const
  {
    return m_links_name;
  }

  const VectorXd& getQMax() const
  {
    return m_q_max;
  }
  const VectorXd getQMin() const
  {
    return m_q_min;
  }
  const VectorXd getDQMax() const
  {
    return m_Dq_max;
  }
  const VectorXd getDDQMax() const
  {
    return m_DDq_max;
  }
  const VectorXd getTauMax() const
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
  template<typename Derived>
  const Eigen::Affine3d& getTransformation(const Eigen::MatrixBase<Derived>& q);

  template<typename Derived>
  const VectorOfAffine3d& getTransformations(const Eigen::MatrixBase<Derived>& q);

  template<typename Derived>
  const Matrix6Xd& getJacobian(const Eigen::MatrixBase<Derived>& q);

  template<typename Derived>
  const VectorOfVector6d& getTwist(const Eigen::MatrixBase<Derived>& q, const Eigen::MatrixBase<Derived>& Dq);

  template<typename Derived>
  const Eigen::Vector6d& getTwistTool(const Eigen::MatrixBase<Derived>& q, const Eigen::MatrixBase<Derived>& Dq)
  {
    return getTwist(q, Dq).back();
  }
  template<typename Derived>
  const VectorOfVector6d& getDTwist(const Eigen::MatrixBase<Derived>& q,
                                      const Eigen::MatrixBase<Derived>& Dq,
                                        const Eigen::MatrixBase<Derived>& DDq);

  template<typename Derived>
  const Eigen::Vector6d& getDTwistTool(const Eigen::MatrixBase<Derived>& q,
                                        const Eigen::MatrixBase<Derived>& Dq,
                                          const Eigen::MatrixBase<Derived>& DDq)
  {
    return getDTwist(q, Dq, DDq).back();
  }

  template<typename Derived>
  const VectorOfVector6d& getDTwistLinearPart(const Eigen::MatrixBase<Derived>& q,
                                                const Eigen::MatrixBase<Derived>& DDq);

  template<typename Derived>
  const Eigen::Vector6d& getDTwistLinearPartTool(const Eigen::MatrixBase<Derived>& q,
                                                   const Eigen::MatrixBase<Derived>& DDq)
  {
    return getDTwistLinearPart(q, DDq).back();
  }

  template<typename Derived>
  const VectorOfVector6d& getDTwistNonLinearPart(const Eigen::MatrixBase<Derived>& q,
                                                  const Eigen::MatrixBase<Derived>& Dq);

  template<typename Derived>
  const Eigen::Vector6d& getDTwistNonLinearPartTool(const Eigen::MatrixBase<Derived>& q,
                                                      const Eigen::MatrixBase<Derived>& Dq)
  {
    return getDTwistNonLinearPart(q, Dq).back();
  }

  template<typename Derived>
  const VectorOfVector6d& getDDTwistLinearPart(const Eigen::MatrixBase<Derived>& q,
                                                const Eigen::VectorXd& DDDq);

  template<typename Derived>
  Eigen::Vector6d getDDTwistLinearPartTool(const Eigen::MatrixBase<Derived>& q,
                                            const Eigen::VectorXd& DDDq)
  {
    return getDDTwistLinearPart(q, DDDq).back();
  }

  template<typename Derived>
  const VectorOfVector6d& getDDTwistNonLinearPart(const Eigen::MatrixBase<Derived>& q,
                                                    const Eigen::MatrixBase<Derived>& Dq,
                                                      const Eigen::MatrixBase<Derived>& DDq);

  template<typename Derived>
  const Eigen::Vector6d& getDDTwistNonLinearPartTool(const Eigen::MatrixBase<Derived>& q,
                                                      const Eigen::MatrixBase<Derived>& Dq,
                                                        const Eigen::MatrixBase<Derived>& DDq)
  {
    return getDDTwistNonLinearPart(q, Dq, DDq).back();
  }

  template<typename Derived>
  const VectorOfVector6d& getDDTwist(const Eigen::MatrixBase<Derived>& q,
                                const Eigen::MatrixBase<Derived>& Dq,
                                  const Eigen::MatrixBase<Derived>& DDq,
                                    const Eigen::VectorXd& DDDq);

  template<typename Derived>
  const Eigen::Vector6d& getDDTwistTool(const Eigen::MatrixBase<Derived>& q,
                                          const Eigen::MatrixBase<Derived>& Dq,
                                            const Eigen::MatrixBase<Derived>& DDq,
                                               const Eigen::MatrixBase<Derived>& DDDq)
  {
    return getDDTwist(q, Dq, DDq, DDDq).back();
  }

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
  template<typename Derived>
  bool computeLocalIk(Eigen::MatrixBase<Derived>& sol,
                        const Eigen::Affine3d& T_b_t,
                          const Eigen::MatrixBase<Derived>& seed,
                            const double& toll = 1e-4,
                              const ros::Duration& max_time = ros::Duration(0.005));

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
  template<typename Derived>
  bool computeWeigthedLocalIk(Eigen::VectorXd& sol, const Eigen::Affine3d& T_b_t,
                                Eigen::Vector6d weight, const Eigen::VectorXd& seed, const double& toll = 1e-4,
                                  const ros::Duration& max_time = ros::Duration(0.005));

  /*
   * Dynamics methods
   */
  template<typename Derived>
  const VectorOfVector6d& getWrench(const Eigen::MatrixBase<Derived>& q,
                                      const Eigen::MatrixBase<Derived>& Dq,
                                        const Eigen::MatrixBase<Derived>& DDq,
                                          VectorOfVector6d& ext_wrenches_in_link_frame);

  template<typename Derived>
  const Eigen::Vector6d& getWrenchTool(const Eigen::MatrixBase<Derived>& q,
                                          const Eigen::MatrixBase<Derived>& Dq,
                                            const Eigen::MatrixBase<Derived>& DDq,
                                              VectorOfVector6d& ext_wrenches_in_link_frame)
  {
    return getWrench(q, DDq, DDq, ext_wrenches_in_link_frame).back();
  }

  template<typename Derived>
  const VectorXd& getJointTorque(const Eigen::MatrixBase<Derived>& q,
                                  const Eigen::MatrixBase<Derived>& Dq,
                                    const Eigen::MatrixBase<Derived>& DDq,
                                      VectorOfVector6d&  ext_wrenches_in_link_frame);

  template<typename Derived>
  const VectorXd& getJointTorque(const Eigen::MatrixBase<Derived>& q,
                                  const Eigen::MatrixBase<Derived>& Dq,
                                    const Eigen::MatrixBase<Derived>& DDq);

  template<typename Derived>
  const VectorXd& getJointTorqueNonLinearPart(const Eigen::MatrixBase<Derived>& q,
                                                const Eigen::MatrixBase<Derived>& Dq);

  template<typename Derived>
  const MatrixXd& getRegressor(const Eigen::MatrixBase<Derived>& q,
                                const Eigen::MatrixBase<Derived>& Dq,
                                  const Eigen::MatrixBase<Derived>& DDq, int* ok = nullptr);

  template<typename Derived>
  const MatrixXd& getJointInertia(const Eigen::MatrixBase<Derived>& q);

  Eigen::VectorXd getNominalParameters();

protected:
  std::vector<rosdyn::Link*> m_links;
  std::vector<rosdyn::Joint*> m_joints;
  unsigned int m_joints_number;
  unsigned int m_active_joints_number;
  unsigned int m_links_number;

  std::vector<std::string> m_links_name;
  std::map<std::string, unsigned int> m_joints_name;
  std::vector<std::string> m_moveable_joints_name;
  std::vector<std::string> m_active_joints_name;

  Matrix6Xd       m_jacobian;

  Eigen::Affine3d m_T_bt;                               // base <- tool

  VectorXd        m_last_q;
  VectorXd        m_sorted_q;

  VectorXd        m_last_Dq;
  VectorXd        m_sorted_Dq;

  VectorXd        m_last_DDq;
  VectorXd        m_sorted_DDq;

  VectorXd        m_last_DDDq;
  VectorXd        m_sorted_DDDq;

  VectorXd        m_q_max;
  VectorXd        m_q_min;
  VectorXd        m_Dq_max;
  VectorXd        m_DDq_max;
  VectorXd        m_tau_max;

  // for QP local ik solver
  MatrixXd        m_CE;
  VectorXd        m_ce0;
  MatrixXd        m_CI;
  VectorXd        m_ci0;
  MatrixXd        m_H;
  VectorXd        m_f;
  VectorXd        m_joint_error;
  Eigen::Vector6d m_cart_error_in_b;

  VectorXd         m_joint_torques;
  VectorXd         m_active_joint_torques;
  ExtendedMatrixXd m_regressor_extended;

  MatrixXd         m_input_to_chain_joint;
  MatrixXd         m_chain_to_input_joint;

  Eigen::Vector3d    m_gravity;
  MatrixXd           m_joint_inertia;
  ExtendedMatrixXd   m_joint_inertia_extended;


  VectorOfAffine3d m_T_bl;
  VectorOfVector6d m_screws_of_c_in_b;
  VectorOfVector6d m_Ds;

  VectorOfVector6d m_twists;    // twists of c in b

  VectorOfVector6d m_Dtwists; // Dtwists of c in b
  VectorOfVector6d m_Dtwists_linear_part;
  VectorOfVector6d m_Dtwists_nonlinear_part;

  VectorOfVector6d m_DDtwists;
  VectorOfVector6d m_DDtwists_linear_part;
  VectorOfVector6d m_DDtwists_nonlinear_part;

  VectorOfVector6d m_wrenches;
  VectorOfVector6d m_inertial_wrenches;
  VectorOfVector6d m_gravity_wrenches;
  VectorOfMatrix610d m_wrenches_regressor;

  MatrixXd         m_regressor_extended_purged;

  std::vector<unsigned int> m_active_joints;

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

  void computeScrews();
  void computeFrames();
};




}

#include <rosdyn_core/internal/primitives_impl.h>

#endif
