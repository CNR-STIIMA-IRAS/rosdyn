#include <ros/ros.h>
#include <rosdyn_utilities/chain_state.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "foo_test");
  ros::NodeHandle nh;

  rosdyn::ChainPtr kin(new rosdyn::Chain());
  rosdyn::ChainStateXPtr state(new rosdyn::ChainStateX(kin));
  rosdyn::ChainState7Ptr state7(new rosdyn::ChainState7(kin) );

  return 0;
}
