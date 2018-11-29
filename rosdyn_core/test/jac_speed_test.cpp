#include <itia_dynamics_core/itia_kin_solver_kdl.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <itia_dynamics_core/itia_urdf_parser.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "itia_kin_solver_test");
  ros::NodeHandle nh;
  ros::Rate rate(1);
  
  itia::dynamics::KinSolverKDL kin_solver("robot_description");
  
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
  
  
  
  itia::JMotion jmotion;
  jmotion.resize(6, 2);
  jmotion.setZero();
  itia::CMotion cmotion;
  
 
  
  itia::Twist twist;
  
  Eigen::Affine3d pose_eigen, pose_kdl;
  
  double t_pose_eigen = 0;
  double t_pose_kdl = 0;
  double t_jac_eigen = 0;
  double t_jac_kdl = 0;
  
  ros::Time t0;
  int ntrial = 1e4;
  
  Eigen::Matrix6Xd jac_eigen, jac_kdl;
  jac_eigen.resize(6, 6);
  jac_kdl.resize(6, 6);
  
  boost::shared_ptr<itia::dynamics::Joint> jptr = root_link->findChildJoint(js.name.at(0));
  for (int idx = 0;idx<ntrial;idx++)
  {
    
    jmotion.col(0) = M_PI*Eigen::Matrix<double, 6, 1>::Random();
    q = jmotion.col(0);
     
    t0 = ros::Time::now();
    pose_eigen = chain->getTransformation(q);
    t_pose_eigen +=  (ros::Time::now()-t0).toSec() *1000;
    
    t0 = ros::Time::now();
    cmotion = kin_solver.fkine(jmotion, tool_frame, base_frame);
    t_pose_kdl +=  (ros::Time::now()-t0).toSec() *1000;
    pose_kdl = itia::dynamics::CMotionToAffine(cmotion, &twist);
    
    t0 = ros::Time::now();
    jac_eigen = chain->getJacobian(q);
    t_jac_eigen +=  (ros::Time::now()-t0).toSec() *1000;
    
    t0 = ros::Time::now();
    jac_kdl = kin_solver.jac(jmotion, tool_frame, base_frame);
    t_jac_kdl +=  (ros::Time::now()-t0).toSec() *1000;

    double err = (jac_eigen-jac_kdl).norm();
    ROS_WARN_STREAM_COND(err>1e-12, "\n JAC error = " << err);
    
    err = (pose_eigen.matrix()-pose_kdl.matrix()).norm();
    ROS_WARN_STREAM_COND(err>1e-6, "\n T error = " << err);
    
  }
  ROS_INFO("computation time POSE EIGEN = %5.4f [ms]",t_pose_eigen/ntrial);
  ROS_INFO("computation time POSE KDL   = %5.4f [ms]",t_pose_kdl/ntrial);
  ROS_INFO("computation time JAC  EIGEN = %5.4f [ms]",t_jac_eigen/ntrial);
  ROS_INFO("computation time JAC  KDL   = %5.4f [ms]",t_jac_kdl/ntrial);
  
  return 0;
}