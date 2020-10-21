#ifndef ROSDYN_UTILITIES__CHAIN_INTERFACE_IMPL__H
#define ROSDYN_UTILITIES__CHAIN_INTERFACE_IMPL__H

#include <sstream>
#include <rosdyn_utilities/chain_interface.h>


#define SP std::fixed  << std::setprecision(5)
#define TP(X) std::fixed << std::setprecision(5) << X.format(m_cfrmt)

namespace rosdyn
{


inline
rosdyn::ChainPtr  ChainInterface::getChain  ( const std::string& from, const std::string& to)
{
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;
  rosdyn::ChainPtr chain = rosdyn::createChain(*m_model,from,to,gravity);
  return chain;
}


inline
bool ChainInterface::init(ros::NodeHandle& nh, 
                          const std::vector<std::string>& joint_names_to_handle,
                          const std::string& base_link,
                          const std::string& tool_link,
                          std::stringstream& report)
{
  try
  {
    std::string robot_description_param;
    std::string robot_description;
    if (!nh.getParam("robot_description_param", robot_description_param ) )
    {
      report <<  nh.getNamespace() + "/robot_description_param/ is not in rosparam server. Superimposed defualt value '/robot_description'" ;
      robot_description_param = "/robot_description";
    }
    if (!nh.getParam(robot_description_param, robot_description))
    {
      report << "Parameter '/robot_description' does not exist";
      return false;
    }

    m_model = urdf::parseURDF(robot_description);
    m_joint_names = joint_names_to_handle;
    m_nAx = m_joint_names.size();

    m_upper_limit    .resize(m_nAx); m_upper_limit  .setZero();
    m_lower_limit    .resize(m_nAx); m_lower_limit  .setZero();
    m_qd_limit       .resize(m_nAx); m_qd_limit     .setZero();
    m_qdd_limit      .resize(m_nAx); m_qdd_limit    .setZero();
    for (unsigned int iAx = 0; iAx < m_nAx; iAx++)
    {
      try
      {
          auto joint_model = m_model->getJoint(m_joint_names.at(iAx));
          if(!joint_model)
          {
            report << "The joint named '" + m_joint_names.at(iAx) + "' is not in the URDF model"; 
            return false;
          }
          m_upper_limit(iAx) = joint_model->limits->upper;
          m_lower_limit(iAx) = joint_model->limits->lower;

          if ((m_upper_limit(iAx) == 0) && (m_lower_limit(iAx) == 0))
          {
            m_upper_limit(iAx) = std::numeric_limits<double>::infinity();
            m_lower_limit(iAx) = -std::numeric_limits<double>::infinity();
            report << "upper and lower limits are both equal to 0, set +/- infinity";
          }

          bool has_velocity_limits;
          if (!nh.getParam(
                "/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/has_velocity_limits", has_velocity_limits))
          {
            has_velocity_limits = false;
          }
          bool has_acceleration_limits;
          if (!nh.getParam(
                "/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/has_acceleration_limits", has_acceleration_limits))
          {
            has_acceleration_limits = false;
          }

          m_qd_limit(iAx) = joint_model->limits->velocity;
          if (has_velocity_limits)
          {
            double vel;
            if (!nh.getParam("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_velocity", vel))
            {
              report << "/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_velocity is not defined";
              return false;
            }
            if (vel < m_qd_limit(iAx))
              m_qd_limit(iAx) = vel;
          }

          if (has_acceleration_limits)
          {
            double acc;
            if (!nh.getParam("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_acceleration", acc))
            {
              report << "/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_acceleration is not defined";
              return false;
            }
            m_qdd_limit(iAx) = acc;
          }
          else
            m_qdd_limit(iAx) = 10 * m_qd_limit(iAx);
      }
      catch (...)
      {
        report << "Unknown excpetion in getting data from joint ''" + m_joint_names.at(iAx) + "'.";
        return false;
      }
    }

    m_base_link = base_link;
    m_tool_link = tool_link;

    Eigen::Vector3d gravity;
    gravity << 0, 0, -9.806;

    std::shared_ptr<rosdyn::Link> root_link(new rosdyn::Link);  //link primitivo da cui parte la catena cinematica (world ad esempio)
    root_link->fromUrdf(m_model->root_link_);

    m_chain.reset(new rosdyn::Chain(root_link, m_base_link, m_tool_link, gravity)); //ricostruisce tutta la catena cinematica andando a leggere l'URDF
    m_chain->setInputJointsName(m_joint_names);
    m_link_names = m_chain->getLinksName( );

    m_cfrmt = Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");

    return true;
  }
  catch(std::exception& e)
  {
    report << "Error caught: " + std::string(e.what());
    return false;
  }
}

inline
bool ChainInterface::saturateSpeed(Eigen::Ref<Eigen::VectorXd> qd_next,
                                     double max_velocity_multiplier,
                                     bool preserve_direction,
                                     std::stringstream* report)
{
  if(report)
  {
    *report << "[-----][SPEED SATURATION] INPUT  qd: " << TP(qd_next.transpose()) << "\n";
  }
  Eigen::VectorXd scale(nAx());
  for(size_t iAx=0;iAx<nAx();iAx++)
  {
    scale(iAx) = std::fabs(qd_next(iAx)) > speedLimit(iAx) * max_velocity_multiplier
               ? speedLimit(iAx) * max_velocity_multiplier/ std::fabs(qd_next(iAx) )
               : 1.0;
  }
  if(preserve_direction)
  {
    qd_next = scale.minCoeff() * qd_next;
  }
  else
  {
    qd_next = scale.asDiagonal() * qd_next;
  }

  if(report)
  {
    *report << (scale.minCoeff()<1 ? "[TRUE ]": "[FALSE]" )
            << "[SPEED SATURATION] OUTPUT qd: " << TP(qd_next.transpose()) << "\n";
  }

  return scale.minCoeff()<1;
}

inline
bool ChainInterface::saturateSpeed(Eigen::Ref<Eigen::VectorXd> qd_next,
                                     const Eigen::Ref<const Eigen::VectorXd> qd_actual,
                                     double dt,
                                     double max_velocity_multiplier,
                                     bool preserve_direction,
                                     std::stringstream* report)
{
  bool saturated = saturateSpeed(qd_next, max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report<<"[-----][ACC   SATURATION] INPUT  qd: "<<TP(qd_next.transpose())
           <<"( qd actual:"<<TP(qd_actual.transpose())<<")\n";
  }
  Eigen::VectorXd qd_sup  = qd_actual + accelerationLimit() * dt;
  Eigen::VectorXd qd_inf  = qd_actual - accelerationLimit() * dt;
  Eigen::VectorXd dqd(nAx()); dqd.setZero();
  for(size_t iAx=0;iAx<nAx();iAx++)
  {
    dqd(iAx) = qd_next(iAx) > qd_sup(iAx) ? (qd_sup(iAx) - qd_next(iAx))
             : qd_next(iAx) < qd_inf(iAx) ? (qd_inf(iAx) - qd_next(iAx))
             : 0.0;
  }
  saturated |= dqd.cwiseAbs().maxCoeff()>0.0;
  if( preserve_direction )
  {
    Eigen::VectorXd dqd_dir = (qd_next - qd_actual).normalized();
    if(dqd.norm() < 1e-5)
    {
      dqd_dir.setZero();
    }

    if(dqd.minCoeff() * dqd.maxCoeff() >= 0.0)
    {
      qd_next = qd_next + (dqd.dot(dqd_dir) * dqd_dir);
    }
    else
    {
      *report << "Target vel     : " << TP(qd_next.transpose()) << "\n";
      *report << "Prev target vel: " << TP(qd_actual.transpose()) << "\n";
      *report << "qd_sup         : " << TP(qd_sup.transpose()) << "\n";
      *report << "qd_inf         : " << TP(qd_inf.transpose()) << "\n";
      *report << "Calc correction: " << TP(dqd.transpose()) << "\n";
      qd_next = qd_next + dqd;
    }
  }


  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            <<"[ACC   SATURATION] OUTPUT qd: "<<TP(qd_next.transpose())<< "\n";
  }
  return saturated;
}

inline
bool ChainInterface::saturateSpeed(Eigen::Ref<Eigen::VectorXd> qd_next,
                                     const Eigen::Ref<const Eigen::VectorXd> qd_actual,
                                     const Eigen::Ref<const Eigen::VectorXd> q_actual,
                                     double dt,
                                     double max_velocity_multiplier,
                                     bool preserve_direction,
                                     std::stringstream* report)
{
  bool saturated = saturateSpeed(qd_next, qd_actual, dt,  max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report << "[-----][BRK   SATURATION] INPUT  qd: " << TP(qd_next.transpose()) 
            << " qd actual: " << TP(qd_actual.transpose()) << "\n";
  }
  Eigen::VectorXd braking_distance(this->nAx());
  for(size_t iAx=0; iAx<this->nAx();iAx++)
  {
    braking_distance(iAx)  = 0.5 * this->accelerationLimit(iAx)
                             * std::pow(std::abs(qd_next(iAx))/this->accelerationLimit(iAx) , 2.0);
  }

  Eigen::VectorXd q_saturated_qd = q_actual + qd_actual* dt;
  for(size_t iAx=0; iAx<this->nAx();iAx++)
  {
    if ((q_saturated_qd(iAx) > (this->upperLimit(iAx) - braking_distance(iAx))) && (qd_next(iAx)>0))
    {
      saturated |= true;
      qd_next(iAx) = std::max(0.0, qd_next(iAx) - this->accelerationLimit(iAx) * dt);
    }
    else if((q_saturated_qd(iAx)<(this->lowerLimit(iAx) + braking_distance(iAx))) && (qd_next(iAx)<0))
    {
      saturated |= true;
      qd_next(iAx) = std::min(0.0, qd_next(iAx) + this->accelerationLimit(iAx) * dt);
    }
  }

  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            << "[BRK   SATURATION] OUTPUT qd: " << TP(qd_next.transpose()) << "\n";
  }
  return saturated;

}

inline
bool ChainInterface::saturatePosition(Eigen::Ref<Eigen::VectorXd> q_next, std::stringstream* report)
{
  if(report)
  {
    *report << "[POS   SATURATION] INPUT  q: " << TP(q_next.transpose()) << "\n";
  }
  
  Eigen::VectorXd dq(q_next.rows());
  for(size_t iAx=0;iAx<nAx();iAx++)
  {
    dq(iAx)  = q_next(iAx) > upperLimit(iAx) ? (upperLimit(iAx) - q_next(iAx))
             : q_next(iAx) < lowerLimit(iAx) ? (lowerLimit(iAx) - q_next(iAx))
             : 0.0;
  }
  
  q_next += dq;
  
  if(report)
  {
    *report << (dq.cwiseAbs().maxCoeff()>0.0 ? "[TRUE ]": "[FALSE]" )
            << "[POS   SATURATION] OUTPUT q: " << TP(q_next.transpose()) << "\n";
  }

  return (dq.cwiseAbs().maxCoeff()>0.0);
}

}  // namespace rosdyn



#undef TP
#undef SP

#endif  // ROSDYN_UTILITIES__CHAIN_INTERFACE_IMPL__H