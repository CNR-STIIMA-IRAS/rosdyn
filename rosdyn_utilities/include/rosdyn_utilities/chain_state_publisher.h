#ifndef ROSYDN_UTILITIES__CHAIN_STATE_PUBLISHER__H
#define ROSYDN_UTILITIES__CHAIN_STATE_PUBLISHER__H

#include <memory>
#include <ros/node_handle.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>
#include <rosdyn_utilities/chain_state_n.h>

namespace rosdyn
{

template<int N>
class ChainStatePublisher
{
private:
  ros::NodeHandle nh_;
  std::shared_ptr<rosdyn::ChainState<N> > chain_;
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

  ChainStatePublisher(const std::string& name, const std::shared_ptr<ChainState<N>>& chain) : nh_("/")
  {
    chain_ = chain;
    pub_.init(nh_, "/log/" + name, 4);
    pub_.msg_.name = chain_->jointNames();
    pub_.msg_.position = std::vector<double>(chain_->nAx(), 0.0);
    pub_.msg_.velocity = std::vector<double>(chain_->nAx(), 0.0);
    pub_.msg_.effort   = std::vector<double>(chain_->nAx(), 0.0);
  }

  void publish()
  {
    if(pub_.trylock())
    {
      Eigen::Matrix<double,N,1>::Map(&pub_.msg_.position[0], chain_->q().size()) = chain_->q();
      Eigen::Matrix<double,N,1>::Map(&pub_.msg_.velocity[0], chain_->qd().size()) = chain_->qd();
      Eigen::Matrix<double,N,1>::Map(&pub_.msg_.effort[0], chain_->effort().size()) = chain_->effort();
      pub_.msg_.header.stamp = ros::Time::now();
      pub_.unlockAndPublish();
    }
  }
};


}  // namespace rosdyn

#endif  // ROSYDN_UTILITIES__CHAIN_STATE_PUBLISHER__H
