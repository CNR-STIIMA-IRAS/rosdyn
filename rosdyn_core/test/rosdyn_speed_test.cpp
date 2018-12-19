#include <rosdyn_core/primitives.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosdyn_core/urdf_parser.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "jacobian_speed_test");
  ros::NodeHandle nh;
  ros::Rate rate(1);
  
  std::string base_frame = "ur10_base_link";
  std::string tool_frame = "ur10_ee_link";                 //"ee_link";
  
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
    
  boost::shared_ptr<rosdyn::Link> root_link(new rosdyn::Link());  
  root_link->fromUrdf(model.root_link_);
  boost::shared_ptr<rosdyn::Chain> chain(new rosdyn::Chain(root_link, base_frame,tool_frame, grav));
  
  chain->setInputJointsName(js.name);
  
  Eigen::VectorXd q(6);
  q.setZero();
  Eigen::VectorXd Dq(6);
  Dq.setZero();
  Eigen::VectorXd DDq(6);
  DDq.setZero();
  Eigen::VectorXd DDDq(6);
  DDDq.setZero();
  Eigen::VectorXd tau(6);
  tau.setZero();
  
  
 
  Eigen::Affine3d pose_eigen;
  std::vector< Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > twists, nonlinacc_twists, linacc_twists, acc_twists, jerk_twists;
  Eigen::Matrix6Xd jac_eigen;
  jac_eigen.resize(6, chain->getActiveJointsNumber());
  Eigen::MatrixXd joint_inertia;
  
  double t_null=0;
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
  
  
  for (int idx = 0;idx<ntrial;idx++)
  {
    
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
  
    t0 = ros::Time::now();
    t_null +=  (ros::Time::now()-t0).toSec() * 1e6;t0 = ros::Time::now();
    
    pose_eigen = chain->getTransformation(q);
    t_pose_eigen +=  (ros::Time::now()-t0).toSec() *1e6;

    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = ros::Time::now();
    jac_eigen = chain->getJacobian(q);
    t_jac_eigen +=  (ros::Time::now()-t0).toSec() *1e6;
  
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = ros::Time::now();
    twists = chain->getTwist(q, Dq);
    t_vel_eigen +=  (ros::Time::now()-t0).toSec() * 1e6;
    
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = ros::Time::now();
    linacc_twists = chain->getDTwistLinearPart(q, DDq);
    t_linacc_eigen +=  (ros::Time::now()-t0).toSec() * 1e6;
    
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = ros::Time::now();
    nonlinacc_twists = chain->getDTwistNonLinearPart(q, Dq);
    t_nonlinacc_eigen +=  (ros::Time::now()-t0).toSec() * 1e6;
    
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = ros::Time::now();
    acc_twists = chain->getDTwist(q, Dq, DDq);
    t_acc_eigen +=  (ros::Time::now()-t0).toSec() * 1e6;
    
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = ros::Time::now();
    jerk_twists = chain->getDDTwist(q, Dq, DDq, DDDq);
    t_jerk_eigen +=  (ros::Time::now()-t0).toSec() * 1e6;
    
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = ros::Time::now();
    tau = chain->getJointTorque(q, Dq, DDq);
    t_torque_eigen +=  (ros::Time::now()-t0).toSec() * 1e6;
    
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    DDDq.setRandom();
    t0 = ros::Time::now();
    joint_inertia = chain->getJointInertia(q);
    t_inertia_eigen +=  (ros::Time::now()-t0).toSec() * 1e6;
    
    
  }
  ROS_INFO("average on %d trials: \nnote:\ncompute torque implies computing acceleration,\ncompute acceleration implies computing velocity,\ncompute velocity implies computing pose",ntrial);
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
  
  return 0;
}