#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <itia_dynamics_core/itia_urdf_parser.h>

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include <err.h>
#include <itia_controllers_and_filters/discrete_state_space.h>
int main(int argc, char **argv){
  ros::init(argc, argv, "itia_kin_solver_test");
  ros::NodeHandle nh;
  double st=2e-3;
  ros::Rate rate(1.0/st);
  
 
  
  itia::rutils::MsgReceiver<sensor_msgs::JointState> m_fb_js_rec;
  ros::Subscriber m_js_fb_sub     = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, &m_fb_js_rec);
  ros::Publisher m_pub=nh.advertise<sensor_msgs::JointState>("/joint_states_model",1);
  sensor_msgs::JointState msg;
  
  if (!m_fb_js_rec.waitForANewData(10))
  {
    ROS_ERROR("/link/joint_states is not available");
    return 0;
  }
  int njnt = m_fb_js_rec.getData().position.size();
  std::vector<std::string> js(njnt);
  js=m_fb_js_rec.getData().name;
  
  
  urdf::Model model;
  model.initParam("robot_description");
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  std::string base_frame = "base";
  std::string tool_frame = "flange";                  //"ee_link"; "upper_arm_link"  "forearm_link"
  std::string model_name=model.getName();
  
  if (!nh.getParam(model_name+"/base_link",base_frame))
  {
    ROS_ERROR("%s/base_link not defined",model_name.c_str());
    return 0;
  }
  if (!nh.getParam(model_name+"/tool_link",tool_frame))
  {
    ROS_ERROR("tool_link not defined");
    return 0;
  }
  
  
  boost::shared_ptr<rosdyn::Link> root_link(new rosdyn::Link());  
  if (!root_link)
    return 0;
  root_link->fromUrdf(model.root_link_);
  boost::shared_ptr<rosdyn::Chain> chain(new rosdyn::Chain(root_link, base_frame,tool_frame, grav));
  
  chain->setInputJointsName(js);
  
  Eigen::VectorXd qnofilt(njnt);
  qnofilt.setZero();
  Eigen::VectorXd Dqnofilt(njnt);
  Dqnofilt.setZero();
  Eigen::VectorXd q(njnt);
  q.setZero();
  Eigen::VectorXd Dq(njnt);
  Dq.setZero();
  Eigen::VectorXd DDq(njnt);
  DDq.setZero();
  
  Eigen::VectorXd tau(njnt);
  tau.setZero();
  
  
  // definisce vettore di filtri
  std::vector<itia::control::DiscreteStateSpacePtr> pos_filter(njnt);
  std::vector<itia::control::DiscreteStateSpacePtr> vel_filter(njnt);
  std::vector<itia::control::DiscreteStateSpacePtr> acc_filter(njnt);
  
  Eigen::VectorXd init(1);
  tau.setZero();
  for (unsigned int ijnt=0;ijnt<njnt;ijnt++)
  {
    std::string filter_name=model_name+ std::string("_filter_info/position_filter");
    pos_filter.at(ijnt)=itia::control::createDiscreteStateSpareFromParam(nh,filter_name ,init);
    filter_name=model_name+ std::string("_filter_info/velocity_filter");
    vel_filter.at(ijnt)=itia::control::createDiscreteStateSpareFromParam(nh,filter_name ,init);
    filter_name=model_name+ std::string("_filter_info/acceleration_filter");
    acc_filter.at(ijnt)=itia::control::createDiscreteStateSpareFromParam(nh,filter_name ,init);
  }
  // definisce vettore di componenti aggiuntivi (Attriti)
  std::vector<rosdyn::ComponentPtr> components;
  try
  {
    for (unsigned int idx = 0;idx <js.size();idx++)
    {
      std::string component_type;
      if (nh.getParam( model_name+"/"+js.at(idx)+"/spring/type", component_type))
      {
        if (!component_type.compare("Ideal"))
        {
          ROS_INFO("JOINT '%s' has a spring component", js.at(idx).c_str());
          components.push_back(rosdyn::ComponentPtr(new rosdyn::IdealSpring(js.at(idx), model_name) ));
        }
      }
      
      if (nh.getParam( model_name+"/"+js.at(idx)+"/friction/type", component_type))
      {
        if (!component_type.compare("Polynomial1"))
        {
          ROS_INFO("JOINT '%s' has a Polynomial1 component", js.at(idx).c_str());
          components.push_back( rosdyn::ComponentPtr(new rosdyn::FirstOrderPolynomialFriction( js.at(idx), model_name ) ));
        } 
        else if (!component_type.compare("Polynomial2"))
        {
          ROS_INFO("JOINT '%s' has a Polynomial2 component", js.at(idx).c_str());
          components.push_back(rosdyn::ComponentPtr(new rosdyn::SecondOrderPolynomialFriction(js.at(idx), model_name) ));
        } 
      }      
    }
  }
  catch (std::exception &e)
  {
    ROS_ERROR("Exception: %s",e.what());
  }
  
  msg.name=js;
  msg.position.resize(njnt);
  msg.velocity.resize(njnt);
  msg.effort.resize(njnt);
  
  while (ros::ok())
  {
    ros::spinOnce();
    
    for (int iAx=0;iAx<njnt;iAx++)
    {
      qnofilt(iAx)=m_fb_js_rec.getData().position.at(iAx);
      Dqnofilt(iAx)=m_fb_js_rec.getData().velocity.at(iAx);
      
      q.block(iAx,0,1,1)=pos_filter.at(iAx)->update(qnofilt.block(iAx,0,1,1));
      Dq.block(iAx,0,1,1)=vel_filter.at(iAx)->update(Dqnofilt.block(iAx,0,1,1));
      DDq.block(iAx,0,1,1)=acc_filter.at(iAx)->update(Dqnofilt.block(iAx,0,1,1));
    }
    // calcola coppia inerziale+Coriolis+gravita'
    tau  = chain->getJointTorque(q, Dq, DDq);  
    
    for (int iC=0;iC<components.size();iC++)
    {
      // calcola coppia componenti aggiuntivi (attriti)
      tau += components.at(iC)->getTorque(q,Dq,DDq);
    }
//     tau=DDq;

    for (int iAx=0;iAx<njnt;iAx++)
    {
      msg.position.at(iAx)=q(iAx);
      msg.velocity.at(iAx)=Dq(iAx);
      msg.effort.at(iAx)=tau(iAx);
    }
    msg.header.stamp=m_fb_js_rec.getData().header.stamp;
    m_pub.publish(msg);
    rate.sleep();
  }
  return 0;
  
}
