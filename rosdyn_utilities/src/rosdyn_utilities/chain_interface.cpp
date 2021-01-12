#include <sstream>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <eigen_state_space_systems/utils/operations.h>
#include <rosdyn_utilities/chain_interface.h>


namespace rosdyn
{


rosdyn::ChainPtr  ChainInterface::getChain  ( const std::string& from, const std::string& to)
{
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;
  rosdyn::ChainPtr chain = rosdyn::createChain(*m_model,from,to,gravity);
  return chain;
}



int ChainInterface::init(ros::NodeHandle& nh, 
                          const std::vector<std::string>& joint_names_to_handle,
                          const std::string& base_link,
                          const std::string& tool_link,
                          std::stringstream& report)
{
  try
  {
    //=============================================================
    std::string urdf_string;
    std::string robot_description_param;
    if(!nh.getParam("robot_description_param", robot_description_param ) )
    {
      report <<  "\nParameter '" + nh.getNamespace() + "/robot_description_param' is not in rosparam server.\n";
      report <<  help();
      return 0;
    }
    else if(!ros::param::has(robot_description_param))
    {
      report << "\nParameter '" << robot_description_param << "' is not in rosparam server.\n";
      report <<  help();
      return -1;
    }
    if(!ros::param::get(robot_description_param, urdf_string))
    {
      report << "\nWeird error in getting the parameter '" << robot_description_param << "'. It was already checked the existence.\n";
      return -1;
    }
    //=============================================================

    //=============================================================
    std::string robot_description_planning_param;
    if(!nh.getParam("robot_description_planning_param", robot_description_planning_param ) )
    {
      report <<  "\nParameter '" + nh.getNamespace() + "/robot_description_planning_param' is not in rosparam server." ;
      report <<  help();
      return 0;
    }
    if(!ros::param::has(robot_description_planning_param))
    {
      report << "\nParameter '" << robot_description_planning_param << "' is not in rosparam server.\n";
      report <<  help();
      return -1;
    }
    //=============================================================



    m_model = urdf::parseURDF(urdf_string);
    if(m_model == nullptr )
    {
      report << "\nParsing the URDF from parameter '" << robot_description_param << "' failed. Weird!\n";
      return -1;
    }
    m_joint_names = joint_names_to_handle;
    m_nAx         = m_joint_names.size();

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
          report << "The joint named '" + m_joint_names.at(iAx) + "' is not in the URDF model."; 
          return -1;
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
        if (!nh.getParam(robot_description_planning_param +"/joint_limits/" + m_joint_names.at(iAx) + "/has_velocity_limits", has_velocity_limits))
        {
          has_velocity_limits = false;
        }
        bool has_acceleration_limits;
        if (!nh.getParam(robot_description_planning_param +"/joint_limits/" + m_joint_names.at(iAx) + "/has_acceleration_limits", has_acceleration_limits))
        {
          has_acceleration_limits = false;
        }

        m_qd_limit(iAx) = joint_model->limits->velocity;
        if (has_velocity_limits)
        {
          double vel;
          if (!nh.getParam(robot_description_planning_param +"/joint_limits/" + m_joint_names.at(iAx) + "/max_velocity", vel))
          {
            report << robot_description_planning_param +"/joint_limits/" + m_joint_names.at(iAx) + "/max_velocity is not defined";
            return 0;
          }
          if (vel < m_qd_limit(iAx))
            m_qd_limit(iAx) = vel;
        }

        if (has_acceleration_limits)
        {
          double acc;
          if (!nh.getParam(robot_description_planning_param +"/joint_limits/" + m_joint_names.at(iAx) + "/max_acceleration", acc))
          {
            report << robot_description_planning_param +"/joint_limits/" + m_joint_names.at(iAx) + "/max_acceleration is not defined";
            return 0;
          }
          m_qdd_limit(iAx) = acc;
        }
        else
          m_qdd_limit(iAx) = 10 * m_qd_limit(iAx);
      }
      catch (...)
      {
        report << "Unknown excpetion in getting data from joint ''" + m_joint_names.at(iAx) + "'.";
        return -1;
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
  }
  catch(std::exception& e)
  {
    report << "Error caught: " + std::string(e.what());
    return -1;
  }
  catch(...)
  {
    report << "Error! ";
    return -1;
  }
  return 1;
}


bool ChainInterface::saturateSpeed(Eigen::Ref<Eigen::VectorXd> qd_next,
                                     double max_velocity_multiplier,
                                     bool preserve_direction,
                                     std::stringstream* report)
{
  if(report)
  {
    *report << "[-----][SPEED SATURATION] INPUT  qd: " << eigen_utils::to_string(qd_next) << "\n";
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
            << "[SPEED SATURATION] OUTPUT qd: " << eigen_utils::to_string(qd_next) << "\n";
  }

  return scale.minCoeff()<1;
}


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
    *report<<"[-----][ACC   SATURATION] INPUT  qd: "<<eigen_utils::to_string(qd_next)<<"\n";
  }
  Eigen::VectorXd qd_sup  = qd_actual + accelerationLimit() * dt;
  Eigen::VectorXd qd_inf  = qd_actual - accelerationLimit() * dt;
  Eigen::VectorXd dqd(nAx()); dqd.setZero();
  for(size_t iAx=0;iAx<nAx();iAx++)
  {
    dqd(iAx) = qd_next(iAx) > qd_sup(iAx) ? (qd_sup(iAx) - qd_next(iAx))
             : qd_next(iAx) < qd_inf(iAx) ? (qd_inf(iAx) + qd_next(iAx))
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
      *report << "Target vel     : " << eigen_utils::to_string(qd_next) << "\n";
      *report << "Prev target vel: " << eigen_utils::to_string(qd_actual) << "\n";
      *report << "qd_sup         : " << eigen_utils::to_string(qd_sup) << "\n";
      *report << "qd_inf         : " << eigen_utils::to_string(qd_inf) << "\n";
      *report << "Calc correction: " << eigen_utils::to_string(dqd) << "\n";
      qd_next = qd_next + dqd;
    }
  }

  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            <<"[ACC   SATURATION] OUTPUT qd: "<<eigen_utils::to_string(qd_next)<< "\n";
  }
  return saturated;
}


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
    *report << "[-----][BRK   SATURATION] INPUT  qd: " << eigen_utils::to_string(qd_next) << "\n";
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
      saturated = true;
      qd_next(iAx) = std::max(0.0, qd_next(iAx) - this->accelerationLimit(iAx) * dt);
    }
    else if((q_saturated_qd(iAx)<(this->lowerLimit(iAx) + braking_distance(iAx))) && (qd_next(iAx)<0))
    {
      saturated = true;
      qd_next(iAx) = std::min(0.0, qd_next(iAx) + this->accelerationLimit(iAx) * dt);
    }
  }

  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            << "[BRK   SATURATION] OUTPUT qd: " << eigen_utils::to_string(qd_next) << "\n";
  }
  return saturated;

}


bool ChainInterface::saturatePosition(Eigen::Ref<Eigen::VectorXd> q_next, std::stringstream* report)
{
  if(report)
  {
    *report << "[POS   SATURATION] INPUT  q: " << eigen_utils::to_string(q_next) << "\n";
  }
  
  Eigen::VectorXd dq(q_next.rows());
  for(size_t iAx=0;iAx<nAx();iAx++)
  {
    dq(iAx)  = q_next(iAx) > upperLimit(iAx) ? (upperLimit(iAx) - q_next(iAx))
             : q_next(iAx) < lowerLimit(iAx) ? (lowerLimit(iAx) + q_next(iAx))
             : 0.0;
  }
  
  q_next += dq;
  
  if(report)
  {
    *report << (dq.cwiseAbs().maxCoeff()>0.0 ? "[TRUE ]": "[FALSE]" )
            << "[POS   SATURATION] OUTPUT q: " << eigen_utils::to_string(q_next) << "\n";
  }

  return (dq.cwiseAbs().maxCoeff()>0.0);
}

bool ChainInterface::saturateSpeed(double& qd_next,
                                   double max_velocity_multiplier,
                                   bool /*preserve_direction*/,
                                   std::stringstream* report)
{
  if(report)
  {
    *report << "[-----][SPEED SATURATION] INPUT  qd: " << eigen_utils::to_string(qd_next) << "\n";
  }
  double scale = std::fabs(qd_next) > speedLimit(0) * max_velocity_multiplier
               ? speedLimit(0) * max_velocity_multiplier/ std::fabs(qd_next)
               : 1.0;

  qd_next = scale * qd_next;

  if(report)
  {
    *report << (scale<1 ? "[TRUE ]": "[FALSE]" )
            << "[SPEED SATURATION] OUTPUT qd: " << eigen_utils::to_string(qd_next) << "\n";
  }

  return scale<1;
}


bool ChainInterface::saturateSpeed(double& qd_next,
                                     const double& qd_actual,
                                     double dt,
                                     double max_velocity_multiplier,
                                     bool preserve_direction,
                                     std::stringstream* report)
{
  bool saturated = saturateSpeed(qd_next, max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report<<"[-----][ACC   SATURATION] INPUT  qd: "<<eigen_utils::to_string(qd_next)<<"\n";
  }
  double qd_sup  = qd_actual + accelerationLimit(0) * dt;
  double qd_inf  = qd_actual - accelerationLimit(0) * dt;
  double dqd = 0;
  dqd  = qd_next > qd_sup ? (qd_sup - qd_next)
       : qd_next < qd_inf ? (qd_inf + qd_next)
       : 0.0;
  saturated |= std::fabs(dqd)>0.0;

  if(report)
  {
    *report << "Target vel     : " << eigen_utils::to_string(qd_next) << "\n";
    *report << "Prev target vel: " << eigen_utils::to_string(qd_actual) << "\n";
    *report << "qd_sup         : " << eigen_utils::to_string(qd_sup) << "\n";
    *report << "qd_inf         : " << eigen_utils::to_string(qd_inf) << "\n";
    *report << "Calc correction: " << eigen_utils::to_string(dqd) << "\n";
  }
  qd_next = qd_next + dqd;

  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            <<"[ACC   SATURATION] OUTPUT qd: "<<eigen_utils::to_string(qd_next)<< "\n";
  }
  return saturated;
}


bool ChainInterface::saturateSpeed(double& qd_next,
                                     const double& qd_actual,
                                     const double& q_actual,
                                     double dt,
                                     double max_velocity_multiplier,
                                     bool preserve_direction,
                                     std::stringstream* report)
{
  bool saturated = saturateSpeed(qd_next, qd_actual, dt,  max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report << "[-----][BRK   SATURATION] INPUT  qd: " << eigen_utils::to_string(qd_next) << "\n";
  }
  double braking_distance;
  braking_distance  = 0.5 * this->accelerationLimit(0)
                           * std::pow(std::abs(qd_next)/this->accelerationLimit(0), 2.0);

  double q_saturated_qd = q_actual + qd_actual* dt;
  if ((q_saturated_qd > (this->upperLimit(0) - braking_distance)) && (qd_next>0))
  {
    saturated = true;
    qd_next = std::max(0.0, qd_next - this->accelerationLimit(0) * dt);
  }
  else if((q_saturated_qd<(this->lowerLimit(0) + braking_distance)) && (qd_next<0))
  {
    saturated = true;
    qd_next = std::min(0.0, qd_next + this->accelerationLimit(0) * dt);
  }

  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            << "[BRK   SATURATION] OUTPUT qd: " << eigen_utils::to_string(qd_next) << "\n";
  }
  return saturated;

}


bool ChainInterface::saturatePosition(double& q_next, std::stringstream* report)
{
  if(report)
  {
    *report << "[POS   SATURATION] INPUT  q: " << eigen_utils::to_string(q_next) << "\n";
  }

  double dq;
  dq   = q_next > upperLimit(0) ? (upperLimit(0) - q_next)
       : q_next < lowerLimit(0) ? (lowerLimit(0) + q_next)
       : 0.0;

  q_next += dq;

  if(report)
  {
    *report << (std::fabs(dq)>0.0 ? "[TRUE ]": "[FALSE]" )
            << "[POS   SATURATION] OUTPUT q: " << eigen_utils::to_string(q_next) << "\n";
  }

  return (std::fabs(dq)>0.0);
}

}  // namespace rosdyn


