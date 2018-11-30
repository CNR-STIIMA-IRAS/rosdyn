#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosdyn_core/urdf_parser.h>
#include <rosdyn_core//primitives.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "itia_inertia_matrix_test");
  ros::NodeHandle nh;
  ros::Rate rate(10);
  
  std::string base_frame = "base";
  std::string tool_frame = "flange";                  //"ee_link"; "upper_arm_link"  "forearm_link"
  
  if (!nh.getParam("base_frame",base_frame))
  {
    ROS_ERROR("base_frame not defined");
  }
  if (!nh.getParam("tool_frame",tool_frame))
  {
    ROS_ERROR("tool_  frame not defined");
  }
  
  
  
  urdf::Model model;
  model.initParam("robot_description");
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;
  
  shared_ptr_namespace::shared_ptr<rosdyn::Link> root_link(new rosdyn::Link());  
  root_link->fromUrdf(model.root_link_);
  shared_ptr_namespace::shared_ptr<rosdyn::Chain> chain(new rosdyn::Chain(root_link, base_frame,tool_frame, grav));
  
  /*chain->setInputJointsName(js);
  
  Eigen::VectorXd q(njnt);
  q.setZero();
  
  while (ros::ok())
  {
    std::string gl;
    
    printf("\n\n%spress a key to test compute inertia \n%s", BOLDWHITE, RESET);
    std::getline(std::cin, gl);
    ros::spinOnce();
    
    for (int iAx=0;iAx<njnt;iAx++)
      q(iAx)=m_fb_js_rec.getData().position.at(iAx);
    
    Eigen::MatrixXd joint_inertia;
    joint_inertia = chain->getJointInertia(q);
    std::cout << "q = "<<  q.transpose() <<std::endl;
    std::cout << "J=\n"<<  joint_inertia <<std::endl;
    rate.sleep();
  }
  */return 0;
  
}
