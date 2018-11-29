#include <itia_dynamics_core/itia_kin_solver_kdl.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <itia_dynamics_core/itia_urdf_parser.h>

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include <err.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "itia_kin_solver_test");
  ros::NodeHandle nh;
  ros::Rate rate(1);
  
  itia::dynamics::KinSolverKDL kin_solver("robot_description");
  
  std::string base_frame = "base_link";
//   std::string tool_frame = "shoulder_link";                  //"ee_link"; "upper_arm_link"  "forearm_link"
//   int njnt = 1;
  
//   std::string tool_frame = "upper_arm_link";                  //"ee_link"; "upper_arm_link"  "forearm_link"
//   int njnt = 2;
  
  std::string tool_frame = "ee_link";                  //"ee_link"; "upper_arm_link"  "forearm_link"
  int njnt = 6;
  
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
  
  std::vector<std::string> js_name(njnt);
  for (int idx = 0;idx<njnt;idx++)
    js_name.at(idx) = js.name.at(idx);
  
  urdf::Model model;
  model.initParam("robot_description");
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;
  
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::Level::Debug);
  boost::shared_ptr<itia::dynamics::Link> root_link(new itia::dynamics::Link());  
  root_link->fromUrdf(model.root_link_);
  boost::shared_ptr<itia::dynamics::Chain> chain(new itia::dynamics::Chain(root_link, base_frame,tool_frame, grav));
  
  
  chain->setInputJointsName(js_name);
  
  Eigen::VectorXd q(njnt);
  q.setZero();
  Eigen::VectorXd Dq(njnt);
  Dq.setZero();
  Eigen::VectorXd DDq(njnt);
  DDq.setZero();
  
  
  Eigen::VectorXd tau(njnt);
  
  KDL::Tree tree;
  if (!kdl_parser::treeFromParam("robot_description", tree))
  {
    ROS_ERROR("tree not created");
    return -1;
  }
  KDL::Chain kdl_chain;
  if (!tree.getChain(base_frame, tool_frame, kdl_chain))
  {
    ROS_ERROR("chain not created");
    return -1;
  }
  KDL::Vector kdl_grav(grav(0), grav(1), grav(2));
  KDL::ChainIdSolver_RNE idsolver(kdl_chain, kdl_grav);
  
  KDL::ChainDynParam kdl_chain_dyn_par(kdl_chain, kdl_grav);
  KDL::JntSpaceInertiaMatrix kdl_joint_inertia;
  
  KDL::JntArray kdl_q, kdl_Dq, kdl_DDq, kdl_tau;
  kdl_q.data.resize(kdl_chain.getNrOfJoints());
  kdl_Dq.data.resize(kdl_chain.getNrOfJoints());
  kdl_DDq.data.resize(kdl_chain.getNrOfJoints());
  kdl_tau.data.resize(kdl_chain.getNrOfJoints());
  kdl_joint_inertia.resize(kdl_chain.getNrOfJoints());
  
  kdl_tau.data.setZero();
  KDL::Wrenches kdl_wrenches(kdl_chain.getNrOfSegments());
  for (unsigned int idx = 0;idx<kdl_chain.getNrOfSegments();idx++)
    kdl_wrenches.at(idx) =KDL::Wrench::Zero();
  
  
  unsigned int ntrial = 0;
  unsigned int errors = 0;
  double tcalc_eigen = 0;
  double tcalc_kdl = 0;
  
  Eigen::MatrixXd joint_inertia;
  
  while (ros::ok())
  {
    ntrial++;
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    
//     chain->getDTwist(q, Dq, DDq);
    ros::Time t0 = ros::Time::now();
    tau = chain->getJointTorque(q, Dq, DDq);
    tcalc_eigen += (ros::Time::now()-t0).toSec();
    joint_inertia = chain->getJointInertia(q);
      
    kdl_q.data = q;
    kdl_Dq.data = Dq;
    kdl_DDq.data = DDq;

    kdl_chain_dyn_par.JntToMass(kdl_q, kdl_joint_inertia);
    
/*
    ROS_INFO_STREAM("J=\n" << joint_inertia);
    ROS_INFO_STREAM("J=\n" << kdl_joint_inertia.data  );
    */
    t0 = ros::Time::now();
    idsolver.CartToJnt(kdl_q, kdl_Dq, kdl_DDq, kdl_wrenches, kdl_tau);
    tcalc_kdl += (ros::Time::now()-t0).toSec();
    
    if ( (kdl_tau.data-tau.block(0, 0, kdl_chain.getNrOfJoints(), 1)).norm()>1e-7 )
    {
      ROS_INFO_STREAM("[ Eigen ] tau = " << tau.transpose());
      ROS_INFO_STREAM("[ KDL   ] tau = " << kdl_tau.data.transpose());
      errors++;
      ROS_ERROR("DIFFERENCE NORM = %e" ,  (kdl_tau.data-tau.block(0, 0, kdl_chain.getNrOfJoints(), 1)).norm());
      ros::Duration(2).sleep();
    }
    else
      ros::Duration(0.0001).sleep();
    
    if ((ntrial % 1000) == 0)
    {
      ROS_WARN("PERC ERROR = %e, tcalc_eigen=%f [ms],  tcalc_kdl =%f [ms] TRIAL =%d", (double)errors/(double)ntrial*100, tcalc_eigen/ntrial*1000, tcalc_kdl/ntrial*1000, ntrial);
      break;
    }
  }
  
  int n_joint_torque = chain->getActiveJointsNumber();
  int npoint = 100000;                                          //5000;
  int n_moveable_links = chain->getLinksNumber()-1;
  
  ros::Time t0 = ros::Time::now();
  Eigen::MatrixXd Phi(npoint*n_joint_torque, 10*n_moveable_links);
  Eigen::VectorXd Tau(npoint*n_joint_torque);
  
  Phi.setZero();
  Tau.setZero();
 
  for (int idx = 0;idx<npoint;idx++)
  {
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    Tau.block(n_joint_torque*idx, 0, n_joint_torque, 1)  = chain->getJointTorque(q, Dq, DDq).block(0, 0, n_joint_torque, 1);  
    Phi.block(n_joint_torque*idx, 0, n_joint_torque, 10*n_moveable_links) = chain->getRegressor(q, Dq, DDq).block(0, 0, n_joint_torque, 10*n_moveable_links);
  }
  
  ROS_INFO("Computational time to build regressors using %d {q, Dq, DDq} n-tuple = %e [ms]", npoint,  (ros::Time::now()-t0).toSec() *1000);
  t0 = ros::Time::now();
  
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Phi,  Eigen::ComputeThinU | Eigen::ComputeThinV);
  
  int rank = 0;
  while ( std::abs(svd.singularValues()(rank))>1e-10 )
  {
    rank++;
    if (rank == (svd.matrixV().rows()))
      break;
  }
  
  ROS_INFO("Computational time to numerically to numerically compute the linear combination of dynamics model parameters (%d parameters) using %d random {q, Dq, DDq} = %e [ms]", rank, npoint,  (ros::Time::now()-t0).toSec() *1000);
  t0 = ros::Time::now();
  Eigen::MatrixXd Phi_numerically_reduced = Phi * svd.matrixV().block(0, 0, svd.matrixV().rows(), rank);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_num_base_phi(Phi_numerically_reduced, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd pi_num_base = svd_num_base_phi.solve(Tau);
  ROS_INFO("Computational time to estimated numerically-computed base parameters using %d {q, Dq, DDq} n-tuple = %e [ms]", npoint,  (ros::Time::now()-t0).toSec() *1000);
//   ROS_INFO_STREAM("Numerically Computed Base Parameters\n" << pi_num_base);
  ROS_INFO("Estimation Error = %4.4e ", (Phi_numerically_reduced*pi_num_base-Tau).norm());
  return 0;
  
}