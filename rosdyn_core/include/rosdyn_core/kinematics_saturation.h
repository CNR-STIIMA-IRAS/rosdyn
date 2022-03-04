#ifndef ROSDYN_CORE__KINEMATICS_SATURATION_H
#define ROSDYN_CORE__KINEMATICS_SATURATION_H

#include <sstream>
#include <Eigen/Dense>
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

}  // namespace rosdyn

#include <rosdyn_core/internal/kinematics_saturation_impl.h>

#endif  // ROSDYN_CORE__KINEMATICS_SATURATION_H
