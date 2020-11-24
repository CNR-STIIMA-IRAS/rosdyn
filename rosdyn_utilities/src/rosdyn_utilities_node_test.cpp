#include <ros/ros.h>
#include <eigen_state_space_systems/filtered_values.h>
#include <rosdyn_utilities/chain_state.h>
#include <rosdyn_utilities/chain_interface.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "foo_test");
  ros::NodeHandle nh;

  rosdyn::ChainInterfacePtr kin(new rosdyn::ChainInterface());
  rosdyn::ChainStatePtr state(new rosdyn::ChainState(kin));

  rosdyn::ChainState7Ptr state7(new rosdyn::ChainState7() );

  return 0;
}