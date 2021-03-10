#ifndef ROSDYN_CORE__KINEMATICS_SATURATION_H
#define ROSDYN_CORE__KINEMATICS_SATURATION_H

#include <Eigen/Dense>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <eigen_matrix_utils/overloads.h>
#include <rosdyn_core/primitives.h>

namespace rosdyn
{


template<typename D1,typename D2,typename D3>
bool saturateSpeed(const rosdyn::Chain& chain,
                   Eigen::MatrixBase<D1>& qd_target,
                   const Eigen::MatrixBase<D2>& qd_actual,
                   const Eigen::MatrixBase<D3>& q_actual,
                   double dt,
                   double max_velocity_multiplier,
                   bool preserve_direction,
                   std::stringstream* report);

template<typename D1,typename D2>
bool saturateSpeed(const rosdyn::Chain& chain,
                   Eigen::MatrixBase<D1>& qd_target,
                   const Eigen::MatrixBase<D2>& qd_actual,
                   double dt,
                   double max_velocity_multiplier,
                   bool preserve_direction,
                   std::stringstream* report);

template<typename D1>
bool saturateSpeed(const rosdyn::Chain& chain,
                   Eigen::MatrixBase<D1>& qd_target,
                   double max_velocity_multiplier,
                   bool preserve_direction,
                   std::stringstream* report);

template<typename D1>
bool saturatePosition(const rosdyn::Chain& chain, Eigen::MatrixBase<D1>& q_target, std::stringstream* report);


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

template <typename D>
using MatD = Eigen::Matrix<double,
                Eigen::MatrixBase<D>::RowsAtCompileTime, Eigen::MatrixBase<D>::ColsAtCompileTime, Eigen::ColMajor,
                  Eigen::MatrixBase<D>::MaxRowsAtCompileTime, Eigen::MatrixBase<D>::MaxColsAtCompileTime>;


template<typename D1>
inline bool saturateSpeed(const rosdyn::Chain& chain,
                            Eigen::MatrixBase<D1>& qd_next,
                              double max_velocity_multiplier,
                                bool preserve_direction,
                                  std::stringstream* report)
{
  if(report)
  {
    *report << "[-----][SPEED SATURATION] INPUT  qd: " << eigen_utils::to_string(qd_next)
               <<" max multiplier: " << max_velocity_multiplier<<"\n";
  }
  MatD<D1> scale;
  eigen_utils::resize(scale, chain.getActiveJointsNumber());

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

template<typename D1, typename D2>
inline bool saturateSpeed(const rosdyn::Chain& chain,
                            Eigen::MatrixBase<D1>& qd_next,
                               const Eigen::MatrixBase<D2>& qd_actual,
                                 double dt,
                                   double max_velocity_multiplier,
                                     bool preserve_direction,
                                       std::stringstream* report)
{
  bool saturated = saturateSpeed(chain, qd_next, max_velocity_multiplier, preserve_direction, report);

  if(report)
  {
    *report<<"[-----][ACC   SATURATION] INPUT  qd: "<<eigen_utils::to_string(qd_next)
              <<" qd actual: " << eigen_utils::to_string(qd_actual)<<"\n";
  }
  MatD<D1> qd_sup, qd_inf, dqd;
  eigen_utils::resize(qd_sup, chain.getActiveJointsNumber());
  eigen_utils::resize(qd_inf, chain.getActiveJointsNumber());
  eigen_utils::resize(dqd   , chain.getActiveJointsNumber());

  qd_sup = qd_actual + chain.getDDQMax() * dt;
  qd_inf  = qd_actual - chain.getDDQMax() * dt;
  eigen_utils::setZero(dqd);
  for(size_t iAx=0;iAx<chain.getActiveJointsNumber();iAx++)
  {
    dqd(iAx) = qd_next(iAx) > qd_sup(iAx) ? (qd_sup(iAx) - qd_next(iAx))
             : qd_next(iAx) < qd_inf(iAx) ? (qd_inf(iAx) - qd_next(iAx))
             : 0.0;
  }
  saturated |= dqd.cwiseAbs().maxCoeff()>1e-5;
  if(saturated)
  {
    if(preserve_direction)
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
        *report << "[-----][ACC   SATURATION] Target vel     : " << eigen_utils::to_string(qd_next) << "\n";
        *report << "[-----][ACC   SATURATION] Prev target vel: " << eigen_utils::to_string(qd_actual) << "\n";
        *report << "[-----][ACC   SATURATION] qd_sup         : " << eigen_utils::to_string(qd_sup) << "\n";
        *report << "[-----][ACC   SATURATION] qd_inf         : " << eigen_utils::to_string(qd_inf) << "\n";
        *report << "[-----][ACC   SATURATION] Calc correction: " << eigen_utils::to_string(dqd) << "\n";
        qd_next = qd_next + dqd;
      }
    }
  }
  else
  {
    // do nothing
  }

  if(report)
  {
    *report << (saturated ? "[TRUE ]": "[FALSE]" )
            <<"[ACC   SATURATION] OUTPUT qd: "<<eigen_utils::to_string(qd_next)<< "\n";
  }
  return saturated;
}

template<typename D1, typename D2, typename D3>
inline bool saturateSpeed(const rosdyn::Chain& chain,
                            Eigen::MatrixBase<D1>&  qd_next,
                              const Eigen::MatrixBase<D2>& qd_actual,
                                const Eigen::MatrixBase<D3>& q_actual,
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
  MatD<D1> braking_distance, q_saturated_qd;
  eigen_utils::resize(braking_distance, chain.getActiveJointsNumber());
  eigen_utils::resize(q_saturated_qd  , chain.getActiveJointsNumber());
  for(size_t iAx=0; iAx<chain.getActiveJointsNumber();iAx++)
  {
    braking_distance(iAx)  = 0.5 * chain.getDDQMax(iAx)
                             * std::pow(std::abs(qd_next(iAx))/chain.getDDQMax(iAx) , 2.0);
  }

  q_saturated_qd = q_actual + qd_actual* dt;
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

template<typename D1>
inline bool saturatePosition(const rosdyn::Chain& chain, Eigen::MatrixBase<D1>& q_next, std::stringstream* report)
{
  if(report)
  {
    *report << "[-----][POS   SATURATION] INPUT  q: " << eigen_utils::to_string(q_next) << "\n";
    *report << "[-----][POS   SATURATION] MAX    q: " << eigen_utils::to_string(chain.getQMax()) << "\n";
    *report << "[-----][POS   SATURATION] MIN    q: " << eigen_utils::to_string(chain.getQMin()) << "\n";
  }

  MatD<D1> dq;
  eigen_utils::resize(dq, q_next.rows());
  for(size_t iAx=0;iAx<chain.getActiveJointsNumber();iAx++)
  {
    dq(iAx)  = q_next(iAx) > chain.getQMax(iAx) ? (chain.getQMax(iAx) - q_next(iAx))
             : q_next(iAx) < chain.getQMin(iAx) ? (chain.getQMin(iAx) - q_next(iAx))
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
    *report << "[-----][SPEED SATURATION] INPUT  qd: " << eigen_utils::to_string(qd_next)
                  <<" max multiplier: " << max_velocity_multiplier<<"\n";
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
    *report << "[-----][POS   SATURATION] INPUT  q: " << eigen_utils::to_string(q_next) << "\n";
    *report << "[-----][POS   SATURATION] MAX    q: " << eigen_utils::to_string(chain.getQMax(0)) << "\n";
    *report << "[-----][POS   SATURATION] MIN    q: " << eigen_utils::to_string(chain.getQMin(0)) << "\n";
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