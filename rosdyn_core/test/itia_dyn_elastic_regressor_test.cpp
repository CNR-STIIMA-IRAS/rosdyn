#include <itia_dynamics_core/itia_kin_solver_kdl.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <itia_dynamics_core/itia_urdf_parser.h>

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <err.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "itia_dyn_elastic_regressor");
  ros::NodeHandle nh;
  ros::Rate rate(1);
  
  std::string base_frame = "base";
  std::string tool_frame = "link2";
  
  
  urdf::Model model;
  model.initParam("robot_description");
  
  
  std::vector<std::string> js_name;
  nh.getParam(model.getName()+"/joint_names", js_name);
  int njnt = js_name.size();

  std::vector<double> grav_stl(3);
  nh.getParam(model.getName()+"/joint_names", grav_stl);
  
  Eigen::Vector3d grav;
  grav << grav_stl.at(0), grav_stl.at(1), grav_stl.at(2);
  
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::Level::Debug);
  
  boost::shared_ptr<itia::dynamics::Link> root_link(new itia::dynamics::Link());  
  root_link->fromUrdf(model.root_link_);
  boost::shared_ptr<itia::dynamics::Chain> chain(new itia::dynamics::Chain(root_link, base_frame,tool_frame, grav));
  
  chain->setInputJointsName(js_name);
  
  std::vector<itia::dynamics::ComponentPtr> components;
  for (unsigned int idx = 0;idx <js_name.size();idx++)
  {
    std::string component_type;
    if (nh.getParam(model.getName()+"/"+js_name.at(idx)+"/spring/type", component_type))
      if (!component_type.compare("Ideal"))
        components.push_back(itia::dynamics::ComponentPtr(new itia::dynamics::IdealSpring(js_name.at(idx), model.getName()) ));
    if (nh.getParam(model.getName()+"/"+js_name.at(idx)+"/friction/type", component_type))
      if (!component_type.compare("Polynomial2"))
        components.push_back(itia::dynamics::ComponentPtr(new itia::dynamics::SecondOrderPolynomialFriction(js_name.at(idx), model.getName()) ));
  }
  
  unsigned int additional_parameters = 0;
  for (unsigned int idx = 0;idx<components.size();idx++)
    additional_parameters += components.at(idx)->getParametersNumber();
  ROS_INFO("Componenent number = %zu,  additional parameters = %u", components.size(), additional_parameters);

  Eigen::VectorXd q(njnt);
  q.setZero();
  Eigen::VectorXd Dq(njnt);
  Dq.setZero();
  Eigen::VectorXd DDq(njnt);
  DDq.setZero();
  
  
  
  Eigen::VectorXd tau(njnt);
  tau.setZero();
  
  int n_joint_number = chain->getActiveJointsNumber();
  int npoint = 1000;                                          //5000;
  int n_moveable_links = chain->getLinksNumber()-1;
  
  ros::Time t0 = ros::Time::now();
  Eigen::MatrixXd Phi(npoint*n_joint_number, (10*n_moveable_links)+additional_parameters);
  Eigen::VectorXd Tau(npoint*n_joint_number);
  Phi.setZero();
  Tau.setZero();


  for (int idx = 0;idx<npoint;idx++)
  {
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();
    Tau.block(n_joint_number*idx, 0, n_joint_number, 1)  = chain->getJointTorque(q, Dq, DDq);
    Phi.block(n_joint_number*idx, 0, n_joint_number, 10*n_moveable_links) = chain->getRegressor(q, Dq, DDq);
    
    unsigned int starting_column = 10*n_moveable_links;
    for (unsigned int iComponent = 0;iComponent<components.size();iComponent++)
    {
      Phi.block(n_joint_number*idx, starting_column, n_joint_number, components.at(iComponent)->getParametersNumber()) = components.at(iComponent)->getRegressor(q, Dq, DDq);
      starting_column += components.at(iComponent)->getParametersNumber();
    }
  }
  ROS_INFO("Computational time to build regressors using %d {q, Dq, DDq} n-tuple = %e [ms]", npoint,  (ros::Time::now()-t0).toSec() *1000);
  t0 = ros::Time::now();
  
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Phi,  Eigen::ComputeThinU | Eigen::ComputeThinV);
  
//   ROS_INFO_STREAM("Singular vuales\n" <<  svd.singularValues());
  int rank = 0;
  while ( std::abs(svd.singularValues()(rank))>1e-10 )
  {
    rank++;
    if (rank == (svd.matrixV().rows()))
      break;
  }
  
  ROS_INFO("Computational time to numerically to numerically compute the linear combination of dynamics model parameters (%d parameters) using %d random {q, Dq, DDq} = %e [ms]", rank, npoint,  (ros::Time::now()-t0).toSec() *1000);

  return 0;
  
}