#include <itia_dynamics_core/itia_kin_solver_kdl.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <itia_dynamics_core/itia_urdf_parser.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "itia_kin_solver_test");
  ros::NodeHandle nh;
  ros::Rate rate(1);
  
  std::string base_frame = "base_link";
  std::string tool_frame = "ee_link";                 //"ee_link";
  
  sensor_msgs::JointState js;
  js.name.resize(6);
  js.position.resize(6);
  js.velocity.resize(6);
  js.name.at(0) = "shoulder_pan_joint";
  js.name.at(1) = "shoulder_lift_joint";
  js.name.at(2) = "elbow_joint";
  js.name.at(3) = "wrist_1_joint";
  js.name.at(4) = "wrist_2_joint";
  js.name.at(5) = "wrist_3_joint";
  
  urdf::Model model;
  model.initParam("robot_description");
  
  
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::Level::Debug);
  boost::shared_ptr<itia::dynamics::Link> root_link(new itia::dynamics::Link());  
  root_link->fromUrdf(model.root_link_);
  boost::shared_ptr<itia::dynamics::Chain> chain(new itia::dynamics::Chain(root_link, base_frame,tool_frame));
  
  chain->setInputJointsName(js.name);
  
  Eigen::VectorXd q(6);
  q.setZero();
  Eigen::VectorXd Dq(6);
  Dq.setZero();
  Eigen::VectorXd DDq(6);
  DDq.setZero();
  Eigen::VectorXd DDDq(6);
  DDDq.setZero();
  
  
  Eigen::Affine3d pose_eigen;
  
  double t_pose_eigen = 0;
  double t_jac_eigen = 0;
  double t_vel_eigen = 0;
  double t_acc_eigen = 0;
  double t_jerk_eigen = 0;
  double t_linacc_eigen = 0;
  double t_nonlinacc_eigen = 0;
  double t_null = 0;
  
  ros::Time t0;
  int ntrial = 1e4;
  
  Eigen::Matrix6Xd jac_eigen;
  jac_eigen.resize(6, 6);
  
  std::vector< Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > twists, nonlinacc_twists, linacc_twists, acc_twists, jerk_twists;
  
  for (int idx = 0;idx<ntrial;idx++)
  {
    
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
     
    t0 = ros::Time::now();
    t_null +=  (ros::Time::now()-t0).toSec() * 1e6;
    
    t0 = ros::Time::now();
    pose_eigen = chain->getTransformation(q);
    t_pose_eigen +=  (ros::Time::now()-t0).toSec() * 1e6;
    
    t0 = ros::Time::now();
    jac_eigen = chain->getJacobian(q);
    t_jac_eigen +=  (ros::Time::now()-t0).toSec() * 1e6;
    
    t0 = ros::Time::now();
    twists = chain->getTwist(q, Dq);
    t_vel_eigen +=  (ros::Time::now()-t0).toSec() * 1e6;

    t0 = ros::Time::now();
    linacc_twists = chain->getDTwistLinearPart(q, DDq);
    t_linacc_eigen +=  (ros::Time::now()-t0).toSec() * 1e6;

    t0 = ros::Time::now();
    nonlinacc_twists = chain->getDTwistNonLinearPart(q, Dq);
    t_nonlinacc_eigen +=  (ros::Time::now()-t0).toSec() * 1e6;

    t0 = ros::Time::now();
    acc_twists = chain->getDTwist(q, Dq, DDq);
    t_acc_eigen +=  (ros::Time::now()-t0).toSec() * 1e6;

    t0 = ros::Time::now();
    jerk_twists = chain->getDDTwist(q, Dq, DDq, DDDq);
    t_jerk_eigen +=  (ros::Time::now()-t0).toSec() * 1e6;
    
    double err = (twists.back()-chain->getJacobian(q) *Dq).norm();
    ROS_WARN_STREAM_COND(err>1e-12, "\n error = " << err);
  }
  
  ROS_INFO("computation time NOP         = %8.5f [us]",t_null/ntrial);
  ROS_INFO("computation time POSE  EIGEN = %8.5f [us]",t_pose_eigen/ntrial);
  ROS_INFO("computation time JAC   EIGEN = %8.5f [us]",t_jac_eigen/ntrial);
  ROS_INFO("computation time VEL   EIGEN = %8.5f [us]",t_vel_eigen/ntrial);
  ROS_INFO("computation time LACC  EIGEN = %8.5f [us]",t_linacc_eigen/ntrial);
  ROS_INFO("computation time NLACC EIGEN = %8.5f [us]",t_nonlinacc_eigen/ntrial);
  ROS_INFO("computation time ACC   EIGEN = %8.5f [us]",t_acc_eigen/ntrial);
  ROS_INFO("computation time JERK  EIGEN = %8.5f [us]",t_jerk_eigen/ntrial);
  
  return 0;
}