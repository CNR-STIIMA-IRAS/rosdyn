#ifndef ROSYDN_UTILITIES__state_STATE_PUBLISHER__H
#define ROSYDN_UTILITIES__state_STATE_PUBLISHER__H

#include <memory>
#include <ros/node_handle.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>
#include <eigen_matrix_utils/overloads.h>
#include <rosdyn_core/primitives.h>
#include <rosdyn_utilities/chain_state.h>

namespace rosdyn
{

template<int N, int MaxN = N>
class ChainStatePublisher
{
private:
  ros::NodeHandle nh_;
  const rosdyn::ChainState<N, MaxN>* state_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> pub_;

public:
  typedef std::shared_ptr<ChainStatePublisher> Ptr;
  typedef std::shared_ptr<ChainStatePublisher const> ConstPtr;

  ChainStatePublisher() = delete;
  virtual ~ChainStatePublisher() = default;
  ChainStatePublisher(const ChainStatePublisher& cpy) = delete;
  ChainStatePublisher(ChainStatePublisher&& cpy) = delete;
  ChainStatePublisher& operator=(const ChainStatePublisher& cpy) = delete;
  ChainStatePublisher& operator=(ChainStatePublisher&& cpy) = delete;

  ChainStatePublisher(const std::string& name, const rosdyn::Chain& chain, const ChainState<N,MaxN>* state)
    : nh_("/"), state_(state)
  {
    if(!state_)
    {
      throw std::runtime_error("Null pointer iput argument!");
    }
    pub_.init(nh_, "/log/" + name, 4);
    pub_.msg_.name = chain.getActiveJointsName();
    pub_.msg_.position = std::vector<double>(chain.getActiveJointsNumber(), 0.0);
    pub_.msg_.velocity = std::vector<double>(chain.getActiveJointsNumber(), 0.0);
    pub_.msg_.effort   = std::vector<double>(chain.getActiveJointsNumber(), 0.0);
  }

  void publish()
  {
    if(pub_.trylock())
    {
      if(state_)
      {
        eigen_utils::copy(pub_.msg_.position, state_->q()     );
        eigen_utils::copy(pub_.msg_.velocity, state_->qd()    );
        eigen_utils::copy(pub_.msg_.effort  , state_->effort());
        pub_.msg_.header.stamp = ros::Time::now();
        pub_.unlockAndPublish();
      }
    }
  }
};

template<int N, int MaxN = N>
using  ChainStatePublisherPtr = typename ChainStatePublisher<N,MaxN>::Ptr;

template<int N, int MaxN = N>
using  ChainStatePublisherConstPtr = typename ChainStatePublisher<N,MaxN>::ConstPtr;

}  // namespace rosdyn

#endif  // ROSYDN_UTILITIES__state_STATE_PUBLISHER__H
