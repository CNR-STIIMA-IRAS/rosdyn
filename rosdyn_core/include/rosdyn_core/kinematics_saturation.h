#ifndef ROSDYN_CORE__KINEMATICS_SATURATION_H
#define ROSDYN_CORE__KINEMATICS_SATURATION_H

#include <Eigen/Dense>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <rosdyn_core/primitives.h>

namespace rosdyn
{


bool saturateSpeed(const rosdyn::Chain& chain,
                   Eigen::Ref<Eigen::VectorXd> qd_target,
                   const Eigen::Ref<const Eigen::VectorXd> qd_actual,
                   const Eigen::Ref<const Eigen::VectorXd> q_actual,
                   double dt,
                   double max_velocity_multiplier,
                   bool preserve_direction,
                   std::stringstream* report);

bool saturateSpeed(const rosdyn::Chain& chain,
                   Eigen::Ref<Eigen::VectorXd> qd_target,
                   const Eigen::Ref<const Eigen::VectorXd> qd_actual,
                   double dt,
                   double max_velocity_multiplier,
                   bool preserve_direction,
                   std::stringstream* report);

bool saturateSpeed(const rosdyn::Chain& chain,
                   Eigen::Ref<Eigen::VectorXd> qd_target,
                   double max_velocity_multiplier,
                   bool preserve_direction,
                   std::stringstream* report);

bool saturatePosition(Eigen::Ref<Eigen::VectorXd> q_target, std::stringstream* report);


//! SPECIAL CASE 1DOF
bool saturateSpeed(const rosdyn::Chain& chain,
                   double& qd_target,
                   const double& qd_actual,
                   const double& q_actual,
                   double dt,
                   double max_velocity_multiplier,
                   bool preserve_direction,
                   std::stringstream* report);

bool saturateSpeed(const rosdyn::Chain& chain,
                   double& qd_target,
                   const double& qd_actual,
                   double dt,
                   double max_velocity_multiplier,
                   bool preserve_direction,
                   std::stringstream* report);

bool saturateSpeed(const rosdyn::Chain& chain,
                   double& qd_target,
                   double max_velocity_multiplier,
                   bool preserve_direction,
                   std::stringstream* report);

bool saturatePosition(const rosdyn::Chain& chain,double& q_target, std::stringstream* report);

}




// implementations


namespace rosdyn
{

inline bool saturateSpeed(const rosdyn::Chain& chain,
                            Eigen::Ref<Eigen::VectorXd> qd_next,
                              double max_velocity_multiplier,
                                bool preserve_direction,
                                  std::stringstream* report)
{
  if(report)
  {
    *report << "[-----][SPEED SATURATION] INPUT  qd: " << eigen_utils::to_string(qd_next) << "\n";
  }
  Eigen::VectorXd scale(chain.getActiveJointsNumber());
  for(size_t iAx=0;iAx<chain.getActiveJointsNumber();iAx++)
  {
    scale(iAx) = std::fabs(qd_next(iAx)) > chain.getDQMax(iAx) * max_velocity_multiplier
               ? chain.getDQMax(iAx) * max_velocity_multiplier/ std::fabs(qd_next(iAx) )
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


inline bool saturateSpeed(const rosdyn::Chain& chain,
                            Eigen::Ref<Eigen::VectorXd> qd_next,
                               const Eigen::Ref<const Eigen::VectorXd> qd_actual,
                                 double dt,
                                   double max_velocity_multiplier,
                                     bool preserve_direction,
                                       std::stringstream* report)
{
  bool saturated = saturateSpeed(chain, qd_next, max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report<<"[-----][ACC   SATURATION] INPUT  qd: "<<eigen_utils::to_string(qd_next)<<"\n";
  }
  Eigen::VectorXd qd_sup  = qd_actual + chain.getDDQMax() * dt;
  Eigen::VectorXd qd_inf  = qd_actual - chain.getDDQMax() * dt;
  Eigen::VectorXd dqd(chain.getActiveJointsNumber()); dqd.setZero();
  for(size_t iAx=0;iAx<chain.getActiveJointsNumber();iAx++)
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


inline bool saturateSpeed(const rosdyn::Chain& chain,
                            Eigen::Ref<Eigen::VectorXd> qd_next,
                              const Eigen::Ref<const Eigen::VectorXd> qd_actual,
                                const Eigen::Ref<const Eigen::VectorXd> q_actual,
                                  double dt,
                                    double max_velocity_multiplier,
                                      bool preserve_direction,
                                        std::stringstream* report)
{
  bool saturated = saturateSpeed(chain, qd_next, qd_actual, dt,  max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report << "[-----][BRK   SATURATION] INPUT  qd: " << eigen_utils::to_string(qd_next) << "\n";
  }
  Eigen::VectorXd braking_distance(chain.getActiveJointsNumber());
  for(size_t iAx=0; iAx<chain.getActiveJointsNumber();iAx++)
  {
    braking_distance(iAx)  = 0.5 * chain.getDDQMax(iAx)
                             * std::pow(std::abs(qd_next(iAx))/chain.getDDQMax(iAx) , 2.0);
  }

  Eigen::VectorXd q_saturated_qd = q_actual + qd_actual* dt;
  for(size_t iAx=0; iAx<chain.getActiveJointsNumber();iAx++)
  {
    if ((q_saturated_qd(iAx) > (chain.getQMax(iAx) - braking_distance(iAx))) && (qd_next(iAx)>0))
    {
      saturated = true;
      qd_next(iAx) = std::max(0.0, qd_next(iAx) - chain.getDDQMax(iAx) * dt);
    }
    else if((q_saturated_qd(iAx)<(chain.getQMin(iAx) + braking_distance(iAx))) && (qd_next(iAx)<0))
    {
      saturated = true;
      qd_next(iAx) = std::min(0.0, qd_next(iAx) + chain.getDDQMax(iAx) * dt);
    }
  }

  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            << "[BRK   SATURATION] OUTPUT qd: " << eigen_utils::to_string(qd_next) << "\n";
  }
  return saturated;

}


inline bool saturatePosition(const rosdyn::Chain& chain, Eigen::Ref<Eigen::VectorXd> q_next, std::stringstream* report)
{
  if(report)
  {
    *report << "[POS   SATURATION] INPUT  q: " << eigen_utils::to_string(q_next) << "\n";
  }

  Eigen::VectorXd dq(q_next.rows());
  for(size_t iAx=0;iAx<chain.getActiveJointsNumber();iAx++)
  {
    dq(iAx)  = q_next(iAx) > chain.getQMax(iAx) ? (chain.getQMax(iAx) - q_next(iAx))
             : q_next(iAx) < chain.getQMin(iAx) ? (chain.getQMin(iAx) + q_next(iAx))
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

inline bool saturateSpeed(const rosdyn::Chain& chain, double& qd_next, double max_velocity_multiplier,
                            bool /*preserve_direction*/, std::stringstream* report)
{
  if(report)
  {
    *report << "[-----][SPEED SATURATION] INPUT  qd: " << eigen_utils::to_string(qd_next) << "\n";
  }
  double scale = std::fabs(qd_next) > chain.getDQMax(0) * max_velocity_multiplier
               ? chain.getDQMax(0) * max_velocity_multiplier/ std::fabs(qd_next)
               : 1.0;

  qd_next = scale * qd_next;

  if(report)
  {
    *report << (scale<1 ? "[TRUE ]": "[FALSE]" )
            << "[SPEED SATURATION] OUTPUT qd: " << eigen_utils::to_string(qd_next) << "\n";
  }

  return scale<1;
}


inline bool saturateSpeed(const rosdyn::Chain& chain, double& qd_next, const double& qd_actual, double dt,
                            double max_velocity_multiplier, bool preserve_direction, std::stringstream* report)
{
  bool saturated = saturateSpeed(chain, qd_next, max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report<<"[-----][ACC   SATURATION] INPUT  qd: "<<eigen_utils::to_string(qd_next)<<"\n";
  }
  double qd_sup  = qd_actual + chain.getDDQMax(0) * dt;
  double qd_inf  = qd_actual - chain.getDDQMax(0) * dt;
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


inline bool saturateSpeed(const rosdyn::Chain& chain, double& qd_next, const double& qd_actual, const double& q_actual,
                          double dt, double max_velocity_multiplier, bool preserve_direction, std::stringstream* report)
{
  bool saturated = saturateSpeed(chain, qd_next, qd_actual, dt,  max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report << "[-----][BRK   SATURATION] INPUT  qd: " << eigen_utils::to_string(qd_next) << "\n";
  }
  double braking_distance;
  braking_distance  = 0.5 * chain.getDDQMax(0)
                           * std::pow(std::abs(qd_next)/chain.getDDQMax(0), 2.0);

  double q_saturated_qd = q_actual + qd_actual* dt;
  if ((q_saturated_qd > (chain.getQMax(0) - braking_distance)) && (qd_next>0))
  {
    saturated = true;
    qd_next = std::max(0.0, qd_next - chain.getDDQMax(0) * dt);
  }
  else if((q_saturated_qd<(chain.getQMin(0) + braking_distance)) && (qd_next<0))
  {
    saturated = true;
    qd_next = std::min(0.0, qd_next + chain.getDDQMax(0) * dt);
  }

  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            << "[BRK   SATURATION] OUTPUT qd: " << eigen_utils::to_string(qd_next) << "\n";
  }
  return saturated;

}


inline bool saturatePosition(const rosdyn::Chain& chain, double& q_next, std::stringstream* report)
{
  if(report)
  {
    *report << "[POS   SATURATION] INPUT  q: " << eigen_utils::to_string(q_next) << "\n";
  }

  double dq;
  dq   = q_next > chain.getQMax(0) ? (chain.getQMax(0) - q_next)
       : q_next < chain.getQMin(0) ? (chain.getQMin(0) + q_next)
       : 0.0;

  q_next += dq;

  if(report)
  {
    *report << (std::fabs(dq)>0.0 ? "[TRUE ]": "[FALSE]" )
            << "[POS   SATURATION] OUTPUT q: " << eigen_utils::to_string(q_next) << "\n";
  }

  return (std::fabs(dq)>0.0);
}
}

#endif
