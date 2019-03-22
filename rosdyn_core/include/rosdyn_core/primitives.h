#ifndef rosdyn_primitivies_201828111025
#define rosdyn_primitivies_201828111025



# include <assert.h>

# include <eigen3/Eigen/Geometry>
# include <eigen3/Eigen/StdVector>


# include <urdf/model.h>
# include <ros/console.h>

# include <rosdyn_core/urdf_parser.h>
# include <rosdyn_core/spacevect_algebra.h>
# include <rosdyn_core/ideal_spring.h>
# include <rosdyn_core/friction_polynomial2.h>
# include <rosdyn_core/friction_polynomial1.h>
# include <rosdyn_core/frame_distance.h>
# include <Eigen/Geometry>
# include<Eigen/StdVector>

#if ROS_VERSION_MINIMUM(1, 14, 1)
# include <memory>
namespace shared_ptr_namespace = std;
#else
# include <boost/concept_check.hpp>
# include <boost/graph/graph_concepts.hpp>
# include <boost/enable_shared_from_this.hpp>
namespace shared_ptr_namespace = boost;
#endif

namespace rosdyn
{
  class Link;
  class Chain;    
  
  class Joint: public shared_ptr_namespace::enable_shared_from_this<rosdyn::Joint>
  {
  protected:
    enum
    {
      REVOLUTE, PRISMATIC, FIXED
    } m_type;
    
    
    
    Eigen::Affine3d m_T_pj;           // transformation parent <- joint
    Eigen::Affine3d m_last_T_pc;      // transformation parent <- child
    Eigen::Matrix3d m_last_R_jc;      // rotatation     joint  <- child
    
    Eigen::Vector6d m_screw_of_c_in_p; // skew of child origin in parent
    
    Eigen::Vector3d m_axis_in_j;
    Eigen::Matrix3d m_skew_axis_in_j;
    Eigen::Matrix3d m_square_skew_axis_in_j;
    Eigen::Matrix3d m_identity;
    
    Eigen::Vector3d m_axis_in_p;
    Eigen::Matrix3d m_skew_axis_in_p;
    Eigen::Matrix3d m_square_skew_axis_in_p;
    Eigen::Matrix3d m_R_pj;
    
    
    double m_last_q; // last value of q
    
    std::string m_name;
    shared_ptr_namespace::shared_ptr<rosdyn::Link> m_parent_link;
    shared_ptr_namespace::shared_ptr<rosdyn::Link> m_child_link;
    
    void computedTpc();
    void computeJacobian();
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Joint();
    void fromUrdf(const shared_ptr_namespace::shared_ptr<urdf::Joint>& urdf_joint, const shared_ptr_namespace::shared_ptr<rosdyn::Link>& parent_link, const shared_ptr_namespace::shared_ptr<urdf::Link>& child_link);
    shared_ptr_namespace::shared_ptr<rosdyn::Joint> pointer();
    std::string getName(){return m_name;};
    shared_ptr_namespace::shared_ptr <rosdyn::Link> getChildLink()  {return m_child_link;}; 
    shared_ptr_namespace::shared_ptr <rosdyn::Link> getParentLink() {return m_parent_link;}; 
    const Eigen::Affine3d& getTransformation(   const double& q = 0);
    const Eigen::Vector6d& getScrew_of_child_in_parent();                      
    
    const bool isFixed() {return (m_type == FIXED);};
  };
  
  class Link: public shared_ptr_namespace::enable_shared_from_this<rosdyn::Link>
  {
  protected:
    shared_ptr_namespace::shared_ptr<rosdyn::Joint> m_parent_joint;
    std::vector<shared_ptr_namespace::shared_ptr<rosdyn::Joint>> m_child_joints;                     
    std::vector<shared_ptr_namespace::shared_ptr<rosdyn::Link>> m_child_links;                     
    std::string m_name;
    
    double m_mass;
    Eigen::Vector3d m_cog_in_c;
    Eigen::Matrix<double, 6, 6> m_Inertia_cc;
    std::vector< Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>> > m_Inertia_cc_single_term;
    
    
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Link() {};
    void fromUrdf(const shared_ptr_namespace::shared_ptr<urdf::Link>& urdf_link, 
                  const shared_ptr_namespace::shared_ptr<rosdyn::Joint>& parent_joint = 0);
    shared_ptr_namespace::shared_ptr<rosdyn::Link> pointer();
    
    std::string getName(){return m_name;};
    shared_ptr_namespace::shared_ptr<rosdyn::Joint> getParentJoint() {return m_parent_joint;};
    std::vector<shared_ptr_namespace::shared_ptr<rosdyn::Joint>> getChildrenJoints() {return m_child_joints;};
    shared_ptr_namespace::shared_ptr<rosdyn::Link> findChild(const std::string& name);
    shared_ptr_namespace::shared_ptr<rosdyn::Joint> findChildJoint(const std::string& name);
    const Eigen::Matrix66d& getSpatialInertia() {return m_Inertia_cc;};
    const std::vector< Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>> >& getSpatialInertiaTerms() {return m_Inertia_cc_single_term;};
    const double& getMass() {return m_mass;};
    const Eigen::Vector3d& getCog() {return m_cog_in_c;};
    Eigen::VectorXd getNominalParameters();
  };
  
  class Chain
  {
  protected:
    std::vector<shared_ptr_namespace::shared_ptr<rosdyn::Link>> m_links;
    std::vector<shared_ptr_namespace::shared_ptr<rosdyn::Joint>> m_joints;
    unsigned int m_joints_number;
    unsigned int m_active_joints_number;
    unsigned int m_links_number;
    
    std::vector<std::string> m_links_name;
    std::map<std::string, unsigned int> m_joints_name;
    std::vector<std::string> m_moveable_joints_name;

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
    
    
    Eigen::VectorXd m_joint_torques;
    Eigen::VectorXd m_active_joint_torques;
    Eigen::MatrixXd m_regressor_extended;
    
    Eigen::MatrixXd m_input_to_chain_joint;
    Eigen::MatrixXd m_chain_to_input_joint;
    std::vector<unsigned int> m_active_joints;
    
    std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > m_T_bl;  //
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
    
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> m_screws_of_c_in_b;
    
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> m_Ds;
    
    // twists of c in b
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> m_twists;
    // Dtwists of c in b
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> m_Dtwists;
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> m_Dtwists_linear_part;
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> m_Dtwists_nonlinear_part;
    
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> m_DDtwists;
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> m_DDtwists_linear_part;
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> m_DDtwists_nonlinear_part;
    
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> m_wrenches;
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> m_inertial_wrenches;
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> m_gravity_wrenches;
    std::vector<Eigen::Matrix<double, 6, 10>,Eigen::aligned_allocator<Eigen::Matrix<double, 6, 10>>> m_wrenches_regressor;
    
    Eigen::Vector3d m_gravity;
    Eigen::MatrixXd m_joint_inertia;
    Eigen::MatrixXd m_joint_inertia_extended;
    
    void computeScrews();
    void computeFrames();
    
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Chain(const shared_ptr_namespace::shared_ptr<rosdyn::Link>& root_link, const std::string& base_link_name, const std::string& ee_link_name, const Eigen::Vector3d& gravity = Eigen::Vector3d::Zero());
    Chain(const urdf::Model& model, const std::string& base_link_name, const std::string& ee_link_name, const Eigen::Vector3d& gravity = Eigen::Vector3d::Zero());
    Chain(const std::string& robot_description, const std::string& base_link_name, const std::string& ee_link_name, const Eigen::Vector3d& gravity = Eigen::Vector3d::Zero());
    void setInputJointsName(const std::vector<std::string> joints_name);
    std::vector<std::string> getMoveableJointNames(){return m_moveable_joints_name;}
    unsigned int getLinksNumber() {return m_links_number;}
    unsigned int getJointsNumber() {return m_joints_number;}
    unsigned int getActiveJointsNumber() {return m_active_joints_number;}
    std::vector<std::string> getLinksName() {return m_links_name;}
    
    /*
     * Kinematics methods
     */
    Eigen::Affine3d getTransformation(const Eigen::VectorXd& q);
    std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> getTransformations(const Eigen::VectorXd& q);
    Eigen::Matrix6Xd getJacobian(const Eigen::VectorXd& q);
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> getTwist(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq);
    Eigen::Vector6d getTwistTool(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq){return getTwist(q,Dq).back();}
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> getDTwist(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq);
    Eigen::Vector6d getDTwistTool(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq){return getDTwist(q,Dq,DDq).back();}
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> getDTwistLinearPart(const Eigen::VectorXd& q, const Eigen::VectorXd& DDq);
    Eigen::Vector6d getDTwistLinearPartTool(const Eigen::VectorXd& q, const Eigen::VectorXd& DDq){return getDTwistLinearPart(q,DDq).back();}
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> getDTwistNonLinearPart(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq);
    Eigen::Vector6d getDTwistNonLinearPartTool(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq){return getDTwistNonLinearPart(q,Dq).back();}
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> getDDTwistLinearPart(const Eigen::VectorXd& q, const Eigen::VectorXd& DDDq);
    Eigen::Vector6d getDDTwistLinearPartTool(const Eigen::VectorXd& q, const Eigen::VectorXd& DDDq){return getDDTwistLinearPart(q,DDDq).back();}
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> getDDTwistNonLinearPart(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq);
    Eigen::Vector6d getDDTwistNonLinearPartTool(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq){return getDDTwistNonLinearPart(q,Dq,DDq).back();}
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> getDDTwist(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq, const Eigen::VectorXd& DDDq);
    Eigen::Vector6d getDDTwistTool(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq, const Eigen::VectorXd& DDDq){return getDDTwist(q,Dq,DDq,DDDq).back();}

    /*
     * Dynamics methods
     */
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> getWrench(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq, std::vector< Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > ext_wrenches_in_link_frame);
    Eigen::Vector6d getWrenchTool(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq, std::vector< Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > ext_wrenches_in_link_frame){return getWrench(q,DDq,DDq,ext_wrenches_in_link_frame).back();}
    Eigen::VectorXd getJointTorque(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq, std::vector< Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > ext_wrenches_in_link_frame);
    Eigen::VectorXd getJointTorque(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq);
    Eigen::VectorXd getJointTorqueNonLinearPart(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq);
    
    Eigen::MatrixXd getRegressor( const Eigen::VectorXd& q, 
                                  const Eigen::VectorXd& Dq, 
                                  const Eigen::VectorXd& DDq);
    
    Eigen::MatrixXd getJointInertia(const Eigen::VectorXd& q);
    Eigen::VectorXd getNominalParameters();
  };
  
  inline Joint::Joint()
  {
    m_last_q = 0;
    
    m_last_T_pc.setIdentity();
    m_last_R_jc.setIdentity();
    
    m_identity.setIdentity();
    m_screw_of_c_in_p.setZero();
    
    computedTpc();
  }
  
  inline void Joint::computeJacobian()
  {
    if (m_type == REVOLUTE)
    {
      m_screw_of_c_in_p << Eigen::MatrixXd::Constant(3, 1, 0), m_axis_in_p;
    }
    else if (m_type == PRISMATIC)
    {
      m_screw_of_c_in_p << m_axis_in_p, Eigen::MatrixXd::Constant(3, 1, 0);
    }
  }
  
  
  inline void Joint::computedTpc()
  {
    if (m_type == REVOLUTE)
    {
      m_last_R_jc = m_identity+sin(m_last_q) *m_skew_axis_in_j+(1-cos(m_last_q)) *m_square_skew_axis_in_j;
      m_last_T_pc.linear().matrix()= m_R_pj*m_last_R_jc;     //m_T_pj+sin(m_last_q) *m_skew_axis_in_p+(1-cos(m_last_q)) *m_square_skew_axis_in_p;
    }
    else if (m_type == PRISMATIC)
      m_last_T_pc.translation() = m_T_pj.translation()+m_axis_in_p*m_last_q;
  }
  
  
  inline void Joint::fromUrdf(const shared_ptr_namespace::shared_ptr< urdf::Joint >& urdf_joint, const shared_ptr_namespace::shared_ptr< Link >& parent_link, const shared_ptr_namespace::shared_ptr< urdf::Link >& child_link)
  {
    m_parent_link = parent_link;  
    
    m_T_pj = urdfPoseToAffine(urdf_joint->parent_to_joint_origin_transform);
    m_axis_in_j = urdfVectorToEigen(urdf_joint->axis);
    
    m_name = urdf_joint->name;
    if (m_axis_in_j.norm()>0)
      m_axis_in_j /= m_axis_in_j.norm();
    
    
    m_skew_axis_in_j = skew(m_axis_in_j);
    m_square_skew_axis_in_j = m_skew_axis_in_j*m_skew_axis_in_j;
    m_identity.setIdentity();
    
    
    // Transform in p frame
    m_R_pj = m_T_pj.linear();
    m_axis_in_p = m_R_pj*m_axis_in_j;
    m_skew_axis_in_p = m_T_pj.linear()*m_skew_axis_in_j;
    m_square_skew_axis_in_p =m_T_pj.linear() *m_square_skew_axis_in_j;
    m_last_T_pc = m_T_pj;
    
    
    if (urdf_joint->type == urdf::Joint::REVOLUTE)
      m_type = rosdyn::Joint::REVOLUTE;
    else if (urdf_joint->type == urdf::Joint::CONTINUOUS)
      m_type = rosdyn::Joint::REVOLUTE;
    else if (urdf_joint->type == urdf::Joint::PRISMATIC)
    {
      m_type = rosdyn::Joint::PRISMATIC;
    }
    else
      m_type = rosdyn::Joint::FIXED;
    
    m_child_link.reset(new rosdyn::Link() );
    m_child_link->fromUrdf(child_link, pointer());
    computedTpc();
    computeJacobian();
    
  }
  
  inline shared_ptr_namespace::shared_ptr<rosdyn::Joint> Joint::pointer()
  {
    return shared_from_this();
  }
  
  inline const Eigen::Affine3d& Joint::getTransformation(const double& q)
  {
    if (q != m_last_q)
    {
      m_last_q = q;
      computedTpc();
    }
    return m_last_T_pc;
  }
  
  
  inline const Eigen::Vector6d& Joint::getScrew_of_child_in_parent()
  {
    return m_screw_of_c_in_p;
  }
  
  
  inline void Link::fromUrdf(const shared_ptr_namespace::shared_ptr<urdf::Link>& urdf_link, const shared_ptr_namespace::shared_ptr<rosdyn::Joint>& parent_joint)
  {
    m_parent_joint = parent_joint;
    m_name = urdf_link->name;
    
    for (unsigned int idx = 0; idx<urdf_link->child_joints.size();idx++)
    {
      m_child_joints.push_back( shared_ptr_namespace::shared_ptr<rosdyn::Joint>(new rosdyn::Joint()));
      m_child_joints.back()->fromUrdf(urdf_link->child_joints.at(idx),pointer(), urdf_link->child_links.at(idx));
      m_child_links.push_back(m_child_joints.at(idx)->getChildLink());
    }
    
    m_mass = 0;
    Eigen::Matrix3d inertia;
    m_Inertia_cc_single_term.resize(10);
    if (urdf_link->inertial != NULL)
    {
      m_mass = urdf_link->inertial->mass;
      
      inertia(0, 0) = urdf_link->inertial->ixx;
      inertia(0, 1) = urdf_link->inertial->ixy;
      inertia(0, 2) = urdf_link->inertial->ixz;
      inertia(1, 0) = urdf_link->inertial->ixy;
      inertia(1, 1) = urdf_link->inertial->iyy;
      inertia(1, 2) = urdf_link->inertial->iyz;
      inertia(2, 0) = urdf_link->inertial->ixz;
      inertia(2, 1) = urdf_link->inertial->iyz;
      inertia(2, 2) = urdf_link->inertial->izz;
      
      m_cog_in_c(0) = urdf_link->inertial->origin.position.x;
      m_cog_in_c(1) = urdf_link->inertial->origin.position.y;
      m_cog_in_c(2) = urdf_link->inertial->origin.position.z;
      
      Eigen::Quaterniond q_p_cog;
      
      q_p_cog.x() = urdf_link->inertial->origin.rotation.x;
      q_p_cog.y() = urdf_link->inertial->origin.rotation.y;
      q_p_cog.z() = urdf_link->inertial->origin.rotation.z;
      q_p_cog.w() = urdf_link->inertial->origin.rotation.w;
      
      // inertia (p) = R_p_cog * inertia_cog * R_cog_p
      inertia = q_p_cog.toRotationMatrix() * inertia * q_p_cog.toRotationMatrix().transpose();
      computeSpatialInertiaMatrix(inertia, m_cog_in_c, m_mass, m_Inertia_cc);
      
      
    }
    else
    {
      inertia.setZero();
      m_cog_in_c.setZero();
      
      computeSpatialInertiaMatrix(inertia, m_cog_in_c, m_mass, m_Inertia_cc);
      for (int idx = 0;idx<10;idx++)
        m_Inertia_cc_single_term.at(idx).setZero();
      
    }
    
    
    
    Eigen::Vector3d mcog;
    Eigen::MatrixXd mcog_skew;
    
    
    //   spatial_inertia.block(0, 0, 3, 3) = mass*Eigen::MatrixXd::Identity(3, 3);
    //   spatial_inertia.block(0, 3, 3, 3) = mass*cog_skew.transpose();
    //   spatial_inertia.block(3, 0, 3, 3) = mass*cog_skew;
    //   spatial_inertia.block(3, 3, 3, 3) = inertia + mass*(cog_skew*cog_skew.transpose());
    
    // mass
    m_Inertia_cc_single_term.at(0).setZero();
    m_Inertia_cc_single_term.at(0).block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
    
    // mcx
    mcog.setZero();
    mcog(0)=1;
    mcog_skew = rosdyn::skew(mcog);
    m_Inertia_cc_single_term.at(1).setZero();
    m_Inertia_cc_single_term.at(1).block(0, 3, 3, 3) = mcog_skew.transpose();
    m_Inertia_cc_single_term.at(1).block(3, 0, 3, 3) = mcog_skew;
    
    // mcy
    mcog.setZero();
    mcog(1)=1;
    mcog_skew = rosdyn::skew(mcog);
    m_Inertia_cc_single_term.at(2).setZero();
    m_Inertia_cc_single_term.at(2).block(0, 3, 3, 3) = mcog_skew.transpose();
    m_Inertia_cc_single_term.at(2).block(3, 0, 3, 3) = mcog_skew;
    
    // mcz
    mcog.setZero();
    mcog(2)=1;
    mcog_skew = rosdyn::skew(mcog);
    m_Inertia_cc_single_term.at(3).setZero();
    m_Inertia_cc_single_term.at(3).block(0, 3, 3, 3) = mcog_skew.transpose();
    m_Inertia_cc_single_term.at(3).block(3, 0, 3, 3) = mcog_skew;
    
    
    
    // Ixx
    m_Inertia_cc_single_term.at(4).setZero();
    m_Inertia_cc_single_term.at(4)(3,3)=1;
    
    // Ixy
    m_Inertia_cc_single_term.at(5).setZero();
    m_Inertia_cc_single_term.at(5)(3,4)=1;
    m_Inertia_cc_single_term.at(5)(4,3)=1;
    
    // Ixz
    m_Inertia_cc_single_term.at(6).setZero();
    m_Inertia_cc_single_term.at(6)(3,5)=1;
    m_Inertia_cc_single_term.at(6)(5,3)=1;
    
    // Iyy
    m_Inertia_cc_single_term.at(7).setZero();
    m_Inertia_cc_single_term.at(7)(4,4)=1;
    
    // Iyz
    m_Inertia_cc_single_term.at(8).setZero();
    m_Inertia_cc_single_term.at(8)(4,5)=1;
    m_Inertia_cc_single_term.at(8)(5,4)=1;
    
    // Iyy
    m_Inertia_cc_single_term.at(9).setZero();
    m_Inertia_cc_single_term.at(9)(5,5)=1;
    
  }
  
  inline Eigen::VectorXd Link::getNominalParameters()
  {
    Eigen::VectorXd nominal_parameters(10);
    
    nominal_parameters(0)=m_mass;
    nominal_parameters.block(1,0,3,1)=m_cog_in_c*m_mass;
    
    Eigen::MatrixXd I0=m_Inertia_cc.block(3,3,3,3);
    nominal_parameters(4)=I0(0,0);
    nominal_parameters(5)=I0(0,1);
    nominal_parameters(6)=I0(0,2);
    
    nominal_parameters(7)=I0(1,1);
    nominal_parameters(8)=I0(1,2);
    
    nominal_parameters(9)=I0(2,2);
    
    return nominal_parameters;
    
    
  }
  
  inline shared_ptr_namespace::shared_ptr<rosdyn::Link> Link::pointer()
  {
    return shared_from_this();
  };
  
  inline shared_ptr_namespace::shared_ptr<rosdyn::Link> Link::findChild(const std::string& name)
  {
    shared_ptr_namespace::shared_ptr<rosdyn::Link> ptr;
    if (!m_name.compare(name))
      return pointer();
    if (m_child_links.size() == 0)
      return ptr;
    for (unsigned int idx = 0;idx<m_child_links.size();idx++)
    {
      if (!m_child_links.at(idx)->getName().compare(name))
        return m_child_links.at(idx);
      ptr = m_child_links.at(idx)->findChild(name);
      if (ptr)
        return ptr;
    }
    return ptr;
  };
  
  inline shared_ptr_namespace::shared_ptr< Joint > Link::findChildJoint(const std::string& name)
  {
    shared_ptr_namespace::shared_ptr<rosdyn::Joint> ptr;
    if (m_child_joints.size() == 0)
      return ptr;
    for (unsigned int idx = 0;idx<m_child_joints.size();idx++)
    {
      if (!m_child_joints.at(idx)->getName().compare(name))
        return m_child_joints.at(idx);
      ptr = m_child_links.at(idx)->findChildJoint(name);
      if (ptr)
        return ptr;
    }
    return ptr;
  }
  
  
  inline Chain::Chain(const shared_ptr_namespace::shared_ptr<rosdyn::Link>& root_link, 
               const std::string& base_link_name, 
               const std::string& ee_link_name, 
               const Eigen::Vector3d& gravity)
  {
    m_is_screws_computed = 
    m_is_jac_computed = 
    m_is_vel_computed = 
    m_is_acc_computed = 
    m_is_nonlinacc_computed = 
    m_is_linacc_computed =
    m_is_jerk_computed = 
    m_is_nonlinjerk_computed = 
    m_is_linjerk_computed =
    m_is_wrench_computed =
    m_is_regressor_computed =
    false;
    
    m_gravity = gravity;
    shared_ptr_namespace::shared_ptr<rosdyn::Link> base_link = root_link->findChild(base_link_name);
    if (!base_link)
    {
      ROS_ERROR("Base link not found");
      return;
    }
    shared_ptr_namespace::shared_ptr<rosdyn::Link> ee_link = base_link->findChild(ee_link_name);
    if (!ee_link)
    {
      ROS_ERROR("Tool link not found");
      return;
    }
    
    shared_ptr_namespace::shared_ptr<rosdyn::Link> act_link(ee_link);
    while (1)
    {
      m_links.insert(m_links.begin(), act_link);
      if (act_link->getName().compare(base_link_name))
      {
        m_joints.insert(m_joints.begin(), act_link->getParentJoint());
        act_link = act_link->getParentJoint()->getParentLink();
      }
      else
        break;
    }
    
    for (unsigned int idx = 0;idx<m_links.size();idx++)
      m_links_name.push_back( m_links.at(idx)->getName() );
    
    for (unsigned int idx = 0;idx<m_joints.size();idx++)
    {
      m_joints_name.insert( std::pair<std::string, unsigned int>(m_joints.at(idx)->getName(), idx) );
      if (!m_joints.at(idx)->isFixed())
        m_moveable_joints_name.push_back(m_joints.at(idx)->getName());
    }

    m_active_joints_number = m_joints_number = m_joints.size();
    m_links_number  = m_links.size();
    m_joint_inertia_extended.resize(m_joints_number, m_joints_number);
    m_joint_inertia_extended.setZero();
    
    
    m_input_to_chain_joint.resize(m_joints_number, m_joints_number);
    m_input_to_chain_joint.setIdentity();
    m_chain_to_input_joint = m_input_to_chain_joint;
    m_T_bt.setIdentity();
    m_last_q.resize(m_joints_number);
    m_last_q.setZero();
    
    m_last_Dq.resize(m_joints_number);
    m_last_Dq.setZero();
    m_last_DDq.resize(m_joints_number);
    m_last_DDq.setZero();
    m_last_DDDq.resize(m_joints_number);
    m_last_DDDq.setZero();
    
    m_active_joints.resize(m_joints_number,1);
    
    m_wrenches_regressor.resize(m_links_number);
    
    
    Eigen::Vector6d zero_vector;
    zero_vector.setZero();
    for (unsigned int idx = 0;idx<m_links_number;idx++)
    {
      m_screws_of_c_in_b.push_back(zero_vector);
      m_twists.push_back(zero_vector);
      m_Dtwists.push_back(zero_vector);
      m_Dtwists_linear_part.push_back(zero_vector);
      m_Dtwists_nonlinear_part.push_back(zero_vector);
      m_DDtwists.push_back(zero_vector);
      m_DDtwists_linear_part.push_back(zero_vector);
      m_DDtwists_nonlinear_part.push_back(zero_vector);
      m_wrenches.push_back(zero_vector);
      m_inertial_wrenches.push_back(zero_vector);
      m_gravity_wrenches.push_back(zero_vector);
      m_wrenches_regressor.at(idx).setZero();
    }
    
    
    m_sorted_q.resize(m_joints_number);
    m_sorted_Dq.resize(m_joints_number);
    m_sorted_DDq.resize(m_joints_number);
    m_sorted_DDDq.resize(m_joints_number);
    m_joint_torques.resize(m_joints_number);
    m_active_joint_torques.resize(m_joints_number);
    m_sorted_q.setZero();
    m_sorted_Dq.setZero();
    m_sorted_DDq.setZero();
    m_sorted_DDDq.setZero();
    m_joint_torques.setZero();
    m_regressor_extended.resize(m_joints_number, m_joints_number*10);
    m_regressor_extended.setZero();
    
    m_jacobian.resize(6, m_joints_number);
    m_jacobian.setZero();
    
    m_T_bl.resize(m_links_number);
    m_T_bl.at(0).setIdentity();
    computeFrames();

    setInputJointsName(m_moveable_joints_name);
  };
  
  inline Chain::Chain(const urdf::Model& model, const std::string& base_link_name, const std::string& ee_link_name, const Eigen::Vector3d& gravity)
  {
    shared_ptr_namespace::shared_ptr<rosdyn::Link> root_link(new rosdyn::Link());
    root_link->fromUrdf(model.root_link_);
    Chain(root_link, base_link_name,ee_link_name, gravity);
  }
  
  inline Chain::Chain(const std::string& robot_description, const std::string& base_link_name, const std::string& ee_link_name, const Eigen::Vector3d& gravity)
  {
    urdf::Model model;
    model.initParam(robot_description);
    shared_ptr_namespace::shared_ptr<rosdyn::Link> root_link(new rosdyn::Link());
    root_link->fromUrdf(model.root_link_);
    Chain(root_link, base_link_name,ee_link_name, gravity);
  }
  
  
  inline void Chain::setInputJointsName(const std::vector< std::string > joints_name)
  {
    m_input_to_chain_joint.resize(m_joints_number, joints_name.size());
    
    m_active_joints_number = joints_name.size();
    m_input_to_chain_joint.setZero();
    m_active_joints.resize(joints_name.size(),0);
    
    //   m_regressor_extended.resize(m_active_joints_number, m_joints_number*10);
    m_regressor_extended.setZero();
    
    m_joint_inertia.resize(m_active_joints_number, m_active_joints_number);
    m_joint_inertia.setZero();
    
    
    for (unsigned int idx = 0;idx<joints_name.size();idx++)
    {
      if (m_joints_name.find(joints_name.at(idx)) != m_joints_name.end())
      {
        m_input_to_chain_joint(m_joints_name.find(joints_name.at(idx))->second, idx) = 1;
        m_active_joints.at(idx) = m_joints_name.find(joints_name.at(idx))->second;
      }
      else
        ROS_WARN("Joint named '%s' not found", joints_name.at(idx).c_str());
	}
      
    m_active_joint_torques.resize(m_active_joints_number);
    m_chain_to_input_joint = m_input_to_chain_joint.transpose();
    
    m_last_q.resize(joints_name.size());
    m_last_q.setZero();
    
    m_sorted_q.setZero();
    m_sorted_Dq.setZero();
    m_sorted_DDq.setZero();
    m_sorted_DDDq.setZero();
    m_jacobian.resize(6, joints_name.size());
    m_jacobian.setZero();
    
    computeFrames();
    
    m_joint_inertia.resize(m_active_joints_number, m_active_joints_number);
    m_joint_inertia.setZero();
    
    Eigen::Matrix4d eye4;
    eye4.setIdentity();
    
    
  }
  
  inline void Chain::computeFrames()
  {
    m_sorted_q = m_input_to_chain_joint*m_last_q;
    for (unsigned int nl = 1;nl<m_links_number;nl++)
    {
      unsigned int nj = nl-1;
      m_T_bl.at(nl).matrix() = m_T_bl.at(nl-1).matrix() * m_joints.at(nj)->getTransformation(m_sorted_q(nj)).matrix();
    }
    m_T_bt = m_T_bl.at(m_links_number-1);
    
  }
  
  inline void Chain::computeScrews()
  {
    for (unsigned int nl = 1;nl<m_links_number;nl++)
    {
      unsigned int nj = nl-1;
      spatialRotation(m_joints.at(nj)->getScrew_of_child_in_parent(), m_T_bl.at(nj).linear().matrix(), &(m_screws_of_c_in_b.at(nl)));
    }
    m_is_screws_computed = true;
  }
  
  inline Eigen::Affine3d Chain::getTransformation(const Eigen::VectorXd& q)
  {  
    if ( (q == m_last_q) || (m_joints_number == 0) )  
      return m_T_bt;
    
    m_last_q = q;
    m_is_screws_computed = 
    m_is_jac_computed = 
    m_is_vel_computed = 
    m_is_acc_computed = 
    m_is_nonlinacc_computed = 
    m_is_linacc_computed =
    m_is_jerk_computed = 
    m_is_nonlinjerk_computed = 
    m_is_linjerk_computed =
    m_is_wrench_computed = 
    m_is_regressor_computed =
    false;
    
    computeFrames();
    
    return m_T_bt;
  }
  
  inline std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> Chain::getTransformations(const Eigen::VectorXd& q)
  {
    getTransformation(q);
    return m_T_bl;
  }

  inline Eigen::Matrix6Xd Chain::getJacobian(const Eigen::VectorXd& q)
  {
    getTransformation(q);
    if (m_joints_number == 0)
      return m_jacobian;
    
    if (m_is_jac_computed)
      return m_jacobian;
    
    if (!m_is_screws_computed)
      computeScrews();
    
    for (unsigned int idx = 0;idx<m_active_joints.size();idx++)
    {
      unsigned int nj = m_active_joints.at(idx);
      unsigned int nl = nj+1;
      if (!m_joints.at(nj)->isFixed())
        m_jacobian.col(idx) = spatialTranslation(m_screws_of_c_in_b.at(nl), m_T_bt.translation()-m_T_bl.at(nl).translation());
    }
    
    m_is_jac_computed = true;
    return m_jacobian;
  }
  
  
  inline std::vector< Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> Chain::getTwist(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq)
  {
    getTransformation(q);
    m_sorted_Dq = m_input_to_chain_joint*Dq;
    if ( (m_sorted_Dq - m_last_Dq).norm()>1e-12)
    {
      m_is_vel_computed = false;
      m_is_acc_computed = false;
      m_is_jerk_computed = false;
      m_is_wrench_computed = false;
      m_is_regressor_computed = false;
      m_is_linacc_computed = false;
      m_is_nonlinacc_computed = false;
      m_is_nonlinjerk_computed = false;
      m_is_linjerk_computed = false;
    }
    m_last_Dq = m_sorted_Dq;
    if (m_is_vel_computed)
      return m_twists;
    
    if (!m_is_screws_computed)
      computeScrews();
    
    for (unsigned int nl = 1;nl<m_links_number;nl++)
    {
      unsigned int nj = nl-1;
      m_twists.at(nl) = spatialTranslation(m_twists.at(nl-1), m_T_bl.at(nl).translation()-m_T_bl.at(nl-1).translation()) + 
      m_screws_of_c_in_b.at(nl) * m_sorted_Dq(nj);
    }
    m_is_vel_computed = true;
    
    return m_twists;
  }
  
  inline std::vector< Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > Chain::getDTwistLinearPart(const Eigen::VectorXd& q, const Eigen::VectorXd& DDq)
  {
    getTransformation(q);
    m_sorted_DDq = m_input_to_chain_joint*DDq;
    if ( (m_sorted_DDq - m_last_DDq).norm()>1e-12)
    {
      m_is_acc_computed = false;
      m_is_jerk_computed = false;
      m_is_wrench_computed = false;
      m_is_regressor_computed = false;
      m_is_linacc_computed = false;
      m_is_nonlinacc_computed = false;
      m_is_nonlinjerk_computed = false;
      m_is_linjerk_computed = false;
    }
    m_last_DDq = m_sorted_DDq;
    
    if (m_is_linacc_computed)
      return m_Dtwists_linear_part;
    
    if (!m_is_screws_computed)
      computeScrews();
    
    for (unsigned int nl = 1;nl<m_links_number;nl++)
    {
      unsigned int nj = nl-1;
      m_Dtwists_linear_part.at(nl) = spatialTranslation(m_Dtwists_linear_part.at(nl-1), m_T_bl.at(nl).translation()-m_T_bl.at(nl-1).translation()) + 
      m_screws_of_c_in_b.at(nl) * m_sorted_DDq(nj);
    }
    m_is_linacc_computed= true;
    
    return m_Dtwists_linear_part;
  };
  
  inline std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> Chain::getDTwistNonLinearPart(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq)
  {
    getTransformation(q);
    if (!m_is_vel_computed)
      getTwist(q, Dq);
    if (m_is_nonlinacc_computed)
      return m_Dtwists_nonlinear_part;
    
    for (unsigned int nl = 1;nl<m_links_number;nl++)
    {
      unsigned int nj = nl-1;
      m_Dtwists_nonlinear_part.at(nl) = spatialTranslation(m_Dtwists_nonlinear_part.at(nl-1), m_T_bl.at(nl).translation()-m_T_bl.at(nl-1).translation()) + 
      spatialCrossProduct(m_twists.at(nl), m_screws_of_c_in_b.at(nl)) * m_sorted_Dq(nj);
    }
    m_is_nonlinacc_computed = true;
    
    return m_Dtwists_nonlinear_part;
  }
  
  inline std::vector< Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > Chain::getDTwist(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq)
  {
    
    getTransformation(q);
    
    m_sorted_DDq = m_input_to_chain_joint*DDq;
    
    if ( (m_sorted_DDq - m_last_DDq).norm()>1e-12)
    {
      m_is_acc_computed = false;
      m_is_jerk_computed = false;
      m_is_wrench_computed = false;
      m_is_regressor_computed = false;
      m_is_linacc_computed = false;
      m_is_nonlinacc_computed = false;
      m_is_nonlinjerk_computed = false;
      m_is_linjerk_computed = false;
    }
    m_last_DDq = m_sorted_DDq;
    
    if (m_is_acc_computed)
      return m_Dtwists;
    if (m_is_linacc_computed && m_is_nonlinacc_computed)
    {
      for (unsigned int nl = 1;nl<m_links_number;nl++)
        m_Dtwists.at(nl) =  m_Dtwists_nonlinear_part.at(nl) + m_Dtwists_linear_part.at(nl);
    }
    else 
    {
      m_sorted_DDq = m_input_to_chain_joint*DDq;
      if (!m_is_vel_computed)
        getTwist(q, Dq);
      for (unsigned int nl = 1;nl<m_links_number;nl++)
      {
        unsigned int nj = nl-1;
        m_Dtwists.at(nl) = spatialTranslation(m_Dtwists.at(nl-1), m_T_bl.at(nl).translation()-m_T_bl.at(nl-1).translation()) + 
        spatialCrossProduct(m_twists.at(nl), m_screws_of_c_in_b.at(nl)) * m_sorted_Dq(nj) +  m_screws_of_c_in_b.at(nl) * m_sorted_DDq(nj);
      }
    }
    
    m_is_acc_computed = true;
    
    return m_Dtwists;
  }
  
  inline std::vector< Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > Chain::getDDTwistLinearPart(const Eigen::VectorXd& q, const Eigen::VectorXd& DDDq)
  {
    getTransformation(q);
    
    m_sorted_DDDq = m_input_to_chain_joint*DDDq;
    if ( (m_sorted_DDDq - m_last_DDDq).norm()>1e-12)
    {
      m_is_jerk_computed = false;
      m_is_nonlinjerk_computed = false;
      m_is_linjerk_computed = false;
    }
    m_last_DDDq = m_sorted_DDDq;
    
    if (m_is_linjerk_computed)
      return m_Dtwists_linear_part;
    
    if (!m_is_screws_computed)
      computeScrews();
    
    for (unsigned int nl = 1;nl<m_links_number;nl++)
    {
      unsigned int nj = nl-1;
      m_DDtwists_linear_part.at(nl) = spatialTranslation(m_DDtwists_linear_part.at(nl-1), m_T_bl.at(nl).translation()-m_T_bl.at(nl-1).translation()) + 
      m_screws_of_c_in_b.at(nl) * m_sorted_DDDq(nj);
    }
    m_is_linjerk_computed = true;
    
    return m_DDtwists_linear_part;
  };
  
  inline std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> Chain::getDDTwistNonLinearPart(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq)
  {
    getTransformation(q);
    if (!m_is_acc_computed)
      getDTwist(q, Dq, DDq);
    
    if (m_is_nonlinjerk_computed)
      return m_DDtwists_nonlinear_part;
    
    /*
     * a(i)= a(i-1)+S(i)*DDq(i)  + v(i) X S(i)*Dq(i) -> DS(i) = v(i) X S(i)
     * j(i)= j(i-1)+S(i)*DDDq(i) + a(i) X S(i)*Dq(i)  + v(i) X S(i)*DDq(i) + v(i) X DS(i)*Dq(i)
     * j(i)= j(i-1)+S(i)*DDDq(i) + v(i) X S(i)*DDq(i) + a(i) X S(i)*Dq(i) + v(i) X v(i) X S(i) * Dq(i)
     */
    Eigen::Vector6d v_cross_s;
    for (unsigned int nl = 1;nl<m_links_number;nl++)
    {
      unsigned int nj = nl-1;
      spatialCrossProduct(m_twists.at(nl), m_screws_of_c_in_b.at(nl), &v_cross_s);
      m_DDtwists_nonlinear_part.at(nl) =  spatialTranslation(m_DDtwists_nonlinear_part.at(nl-1), m_T_bl.at(nl).translation()-m_T_bl.at(nl-1).translation()) + 
      v_cross_s * m_sorted_DDq(nj) +   /* v(i) X S(i)*DDq(i) */
      ( spatialCrossProduct( m_Dtwists.at(nl), m_screws_of_c_in_b.at(nl) ) + /* a(i) X S(i)*Dq(i) */
      spatialCrossProduct( m_twists.at(nl), v_cross_s ) ) * m_sorted_Dq(nj); /* v(i) X v(i) X S(i) * Dq(i) */
      
    }
    m_is_nonlinjerk_computed = true;
    
    return m_DDtwists_nonlinear_part;
  }
  
  inline std::vector< Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > Chain::getDDTwist(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq, const Eigen::VectorXd& DDDq)
  {
    getTransformation(q);
    m_sorted_DDDq = m_input_to_chain_joint*DDDq;
    if ( (m_sorted_DDDq - m_last_DDDq).norm()>1e-12)
    {
      m_is_jerk_computed = false;
      m_is_nonlinjerk_computed = false;
      m_is_linjerk_computed = false;
    }
    m_last_DDDq = m_sorted_DDDq;
    
    if (m_is_jerk_computed)
      return m_DDtwists;
    if (m_is_linjerk_computed && m_is_nonlinjerk_computed)
    {
      for (unsigned int nl = 1;nl<m_links_number;nl++)
        m_DDtwists.at(nl) =  m_DDtwists_nonlinear_part.at(nl) + m_DDtwists_linear_part.at(nl);
    }
    else 
    {
      m_sorted_DDDq = m_input_to_chain_joint*DDDq;
      if (!m_is_acc_computed)
        getDTwist(q, Dq, DDq);
      Eigen::Vector6d v_cross_s;
      for (unsigned int nl = 1;nl<m_links_number;nl++)
      {
        unsigned int nj = nl-1;
        spatialCrossProduct(m_twists.at(nl), m_screws_of_c_in_b.at(nl), &v_cross_s);
        m_DDtwists.at(nl) = spatialTranslation( m_DDtwists.at(nl-1), m_T_bl.at(nl).translation()-m_T_bl.at(nl-1).translation() ) + 
        m_screws_of_c_in_b.at(nl) * m_sorted_DDDq(nj) +
        v_cross_s * m_sorted_DDq(nj) +   /* v(i) X S(i)*DDq(i) */
        ( spatialCrossProduct( m_Dtwists.at(nl), m_screws_of_c_in_b.at(nl) ) + /* a(i) X S(i)*Dq(i) */
        spatialCrossProduct( m_twists.at(nl), v_cross_s ) ) * m_sorted_Dq(nj); /* v(i) X v(i) X S(i) * Dq(i) */
      }
    }
    m_is_jerk_computed = true;
    return m_DDtwists;
  }
  
  inline std::vector< Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > Chain::getWrench(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq, std::vector< Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > ext_wrenches_in_link_frame)
  {
    getDTwist(q, Dq, DDq);
    if (m_is_wrench_computed)
      return m_wrenches;
    
    for (int nl = (m_links_number-1);nl >= 0;nl--)
    { 
      if (nl == 0)
      {
        m_inertial_wrenches.at(nl).setZero();
        m_gravity_wrenches.at(nl).setZero();
      } 
      else
      {
        m_inertial_wrenches.at(nl) = spatialRotation(
          m_links.at(nl)->getSpatialInertia() * 
          spatialRotation(m_Dtwists.at(nl), m_T_bl.at(nl).linear().transpose())
          + 
          spatialDualCrossProduct(
            spatialRotation(m_twists.at(nl), m_T_bl.at(nl).linear().transpose()), 
                                  m_links.at(nl)->getSpatialInertia() * 
                                  spatialRotation(m_twists.at(nl), m_T_bl.at(nl).linear().transpose())
          ), m_T_bl.at(nl).linear());
        m_gravity_wrenches.at(nl).block(0, 0, 3, 1) = -m_links.at(nl)->getMass() *m_gravity;
        m_gravity_wrenches.at(nl).block(3, 0, 3, 1) = -(m_T_bl.at(nl).linear() * m_links.at(nl)->getCog() ).cross(m_links.at(nl)->getMass() * m_gravity);
        
        
      }
      
      if (nl < (int)(m_links_number-1))
        m_wrenches.at(nl) = spatialTranformation(ext_wrenches_in_link_frame.at(nl), m_T_bl.at(nl)) + m_inertial_wrenches.at(nl) + m_gravity_wrenches.at(nl) + spatialDualTranslation( m_wrenches.at(nl+1) , m_T_bl.at(nl).translation()-m_T_bl.at(nl+1).translation() );
      else
        m_wrenches.at(nl) = spatialTranformation(ext_wrenches_in_link_frame.at(nl), m_T_bl.at(nl)) + m_inertial_wrenches.at(nl) + m_gravity_wrenches.at(nl);
      
    }
    
    m_is_wrench_computed = true;
    return m_wrenches;
  }
  
  inline Eigen::VectorXd Chain::getJointTorque(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq, std::vector< Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > ext_wrenches_in_link_frame)
  {
    getWrench(q, Dq, DDq, ext_wrenches_in_link_frame);
    for (unsigned int nj = 0; nj < m_joints_number; nj++)
    {
      unsigned int nl = nj+1;
      m_joint_torques(nj) = m_wrenches.at(nl).dot(m_screws_of_c_in_b.at(nl));
    }
    m_active_joint_torques = m_chain_to_input_joint * m_joint_torques;
    return m_active_joint_torques;
  }
  
  
  inline Eigen::VectorXd Chain::getJointTorque(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq, const Eigen::VectorXd& DDq)
  {
    std::vector< Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > ext_wrenches_in_link_frame(m_links_number);
    for (unsigned int iL = 0;iL < m_links_number; iL++)
      ext_wrenches_in_link_frame.at(iL).setZero();
    return getJointTorque(q, Dq, DDq, ext_wrenches_in_link_frame);
  }
  
  inline Eigen::VectorXd Chain::getJointTorqueNonLinearPart(const Eigen::VectorXd& q, const Eigen::VectorXd& Dq)
  {
    Eigen::VectorXd DDq(m_active_joints_number);
    DDq.setZero();
    std::vector< Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > ext_wrenches_in_link_frame(m_links_number);
    for (unsigned int iL = 0;iL < m_links_number; iL++)
      ext_wrenches_in_link_frame.at(iL).setZero();
    return getJointTorque(q, Dq, DDq, ext_wrenches_in_link_frame);
  }
  
  inline Eigen::MatrixXd Chain::getRegressor(const Eigen::VectorXd& q, 
                                      const Eigen::VectorXd& Dq, 
                                      const Eigen::VectorXd& DDq)
  {
    if (q.rows() != Dq.rows() )
    {
      ROS_ERROR("Input data dimensions mismatch");
      throw std::invalid_argument("Input data dimensions mismatch");
    } 
    
    if (Dq.rows() != DDq.rows() )
    {
      ROS_ERROR("Input data dimensions mismatch");
      throw std::invalid_argument("Input data dimensions mismatch");
    }  
    
    getDTwist(q, Dq, DDq);
    
    if (m_is_regressor_computed)
    {
      static bool verbose_ = true;
      if (verbose_)
        ROS_DEBUG("Regressor input element equals to the previous call. Returned the same dynamics Regressor");
      
      return m_chain_to_input_joint * m_regressor_extended;
    }
    for (int nl = (m_links_number-1);nl > 0;nl--)
    {
      // m, mcx, mcy, mcz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz
      for (unsigned int iPar = 0;iPar<10;iPar++)
      {
        m_wrenches_regressor.at(nl).col(iPar) = spatialRotation(m_links.at(nl)->getSpatialInertiaTerms().at(iPar) * 
        spatialRotation(m_Dtwists.at(nl), m_T_bl.at(nl).linear().transpose()) + 
        spatialDualCrossProduct(
          spatialRotation(m_twists.at(nl), m_T_bl.at(nl).linear().transpose()), 
                                m_links.at(nl)->getSpatialInertiaTerms().at(iPar) * 
                                spatialRotation(m_twists.at(nl), m_T_bl.at(nl).linear().transpose())
        ), m_T_bl.at(nl).linear());
      }
      
      
      m_wrenches_regressor.at(nl).col(0).block(0, 0, 3, 1) -=  m_gravity;
      m_wrenches_regressor.at(nl).col(1).block(3, 0, 3, 1) -=  (m_T_bl.at(nl).linear() * Eigen::Vector3d::UnitX() ).cross(m_gravity);
      m_wrenches_regressor.at(nl).col(2).block(3, 0, 3, 1) -=  (m_T_bl.at(nl).linear() * Eigen::Vector3d::UnitY() ).cross(m_gravity);
      m_wrenches_regressor.at(nl).col(3).block(3, 0, 3, 1) -=  (m_T_bl.at(nl).linear() * Eigen::Vector3d::UnitZ() ).cross(m_gravity);
      
      m_regressor_extended.block(nl-1, (nl-1)*10, 1, 10) = m_screws_of_c_in_b.at(nl).transpose() * m_wrenches_regressor.at(nl);
      
      for (unsigned int nl_following = nl+1; nl_following<m_links_number; nl_following++)
      {
        for (unsigned int iPar = 0;iPar<10;iPar++)
          m_regressor_extended(nl-1, (nl_following-1)*10+iPar) = m_screws_of_c_in_b.at(nl).transpose() * spatialDualTranslation( m_wrenches_regressor.at(nl_following).col(iPar) , m_T_bl.at(nl).translation() - m_T_bl.at(nl_following).translation() );
      }      
    }
    
    Eigen::MatrixXd result;
    // result=m_chain_to_input_joint*result;    //BUG
    result=(m_regressor_extended.transpose()*m_chain_to_input_joint.transpose()).transpose();    
    m_is_regressor_computed = true;
    return result;
  }
  
  inline Eigen::MatrixXd Chain::getJointInertia(const Eigen::VectorXd& q)
  {
    getTransformation(q);
    computeScrews();
    m_joint_inertia_extended.setZero();
    for (unsigned int nj = 0; nj<m_joints_number; nj++)
    {
      Eigen::Matrix6Xd jacobian_nl(6, m_joints_number);
      jacobian_nl.setZero();
      for (unsigned int ij = 0;ij <= nj/*m_active_joints.size()*/;ij++)
      {
        unsigned int il = ij+1;
        if (!m_joints.at(ij)->isFixed())
        {
          jacobian_nl.col(ij) = spatialTranslation(m_screws_of_c_in_b.at(il), m_T_bl.at(nj+1).translation()-m_T_bl.at(il).translation());
          jacobian_nl.col(ij) = spatialRotation(jacobian_nl.col(ij), m_T_bl.at(nj+1).linear().transpose());
        }
      }
      m_joint_inertia_extended += jacobian_nl.transpose() * m_links.at(nj+1)->getSpatialInertia() * jacobian_nl;
    }
    m_joint_inertia = m_chain_to_input_joint * m_joint_inertia_extended * m_input_to_chain_joint;
    return m_joint_inertia;
  };
  
  
  inline Eigen::VectorXd Chain::getNominalParameters()
  {
    
    Eigen::VectorXd nominal_par(10*(m_links_number-1));
    nominal_par.setZero();
    for (int nl = (m_links_number-1);nl > 0;nl--)
    {
      nominal_par.block(10*(nl-1),0,10,1)= m_links.at(nl)->getNominalParameters();
    }
    return nominal_par;
  };


  
  inline boost::shared_ptr<Chain> createChain(const urdf::Model& urdf_model, const std::string& base_frame, const std::string& tool_frame, const Eigen::Vector3d& gravity)
  {
    
    shared_ptr_namespace::shared_ptr<rosdyn::Link> root_link(new rosdyn::Link());  
    root_link->fromUrdf(urdf_model.root_link_);
    boost::shared_ptr<rosdyn::Chain> chain(new rosdyn::Chain(root_link, base_frame,tool_frame, gravity));

    return chain;
  };
  
}

# endif



/*
 * W_ii=[S_ii, vi;0 0]
 * T_ji=[R_ji, p_ji;0 1]
 * W_jj=T_ji*W_ii*T_ij
 * 
 * T_ij=[R_ji', -R_ji'*p_ji;0 1]
 * W_ii*T_ij = [S_ii*R_ji', -S_ii*R_ji'*p_ji+vi;0 0]
 * Rji*W_ii*T_ij = [R_ji*S_ii*R_ji', -R_ji*S_ii*R_ji'*p_ji+R_ji*vi;0 0]
 * Rji*W_ii*T_ij = [S_jj, -S_jj*p_ji+R_ji*vi;0 0]
 */
