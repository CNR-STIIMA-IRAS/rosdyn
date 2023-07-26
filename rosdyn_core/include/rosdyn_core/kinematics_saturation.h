#ifndef ROSDYN_CORE__KINEMATICS_SATURATION_H
#define ROSDYN_CORE__KINEMATICS_SATURATION_H

#include <rdyn_core/kinematics_saturation.h>

namespace rosdyn
{

template<typename D1,typename D2,typename D3>
[[deprecated("Include rdyn_core/kinematics_saturation.h and use namespace rdyn")]]
inline bool saturateSpeed(const rdyn::Chain& chain,
                   Eigen::MatrixBase<D1>& qd_target,
                   const Eigen::MatrixBase<D2>& qd_actual,
                   const Eigen::MatrixBase<D3>& q_actual,
                   double dt,
                   double max_velocity_multiplier,
                   bool preserve_direction,
                   std::stringstream* report)
{ return rdyn::saturateSpeed<D1,D2,D3>(chain,qd_target,qd_actual,q_actual,dt,max_velocity_multiplier,preserve_direction,report); }

template<typename D1,typename D2>
[[deprecated("Include rdyn_core/kinematics_saturation.h and use namespace rdyn")]]
inline bool saturateSpeed(const rdyn::Chain& chain,
                   Eigen::MatrixBase<D1>& qd_target,
                   const Eigen::MatrixBase<D2>& qd_actual,
                   double dt,
                   double max_velocity_multiplier,
                   bool preserve_direction,
                   std::stringstream* report)
{ return rdyn::saturateSpeed<D1,D2>(chain,qd_target,qd_actual,dt,max_velocity_multiplier,preserve_direction,report); }

template<typename D1>
[[deprecated("Include rdyn_core/kinematics_saturation.h and use namespace rdyn")]]
inline bool saturateSpeed(const rdyn::Chain& chain,
                   Eigen::MatrixBase<D1>& qd_target,
                   double max_velocity_multiplier,
                   bool preserve_direction,
                   std::stringstream* report)
{ return rdyn::saturateSpeed<D1>(chain,qd_target,max_velocity_multiplier,preserve_direction,report); }

template<typename D1>
[[deprecated("Include rdyn_core/kinematics_saturation.h and use namespace rdyn")]]
inline bool saturatePosition(const rdyn::Chain& chain, Eigen::MatrixBase<D1>& q_target, std::stringstream* report)
{ return rdyn::saturatePosition<D1>(chain,q_target,report); }


//! SPECIAL CASE 1DOF
[[deprecated("Include rdyn_core/kinematics_saturation.h and use namespace rdyn")]]
inline bool saturateSpeed(const rdyn::Chain& chain,
                   double& qd_target,
                   const double& qd_actual,
                   const double& q_actual,
                   double dt,
                   double max_velocity_multiplier,
                   bool preserve_direction,
                   std::stringstream* report)
{ return rdyn::saturateSpeed(chain,qd_target,qd_actual,q_actual,dt,max_velocity_multiplier,preserve_direction,report); }

[[deprecated("Include rdyn_core/kinematics_saturation.h and use namespace rdyn")]]
inline bool saturateSpeed(const rdyn::Chain& chain,
                   double& qd_target,
                   const double& qd_actual,
                   double dt,
                   double max_velocity_multiplier,
                   bool preserve_direction,
                   std::stringstream* report)
{ return rdyn::saturateSpeed(chain,qd_target,qd_actual,dt,max_velocity_multiplier,preserve_direction,report); }

[[deprecated("Include rdyn_core/kinematics_saturation.h and use namespace rdyn")]]
bool saturateSpeed(const rdyn::Chain& chain,
                   double& qd_target,
                   double max_velocity_multiplier,
                   bool preserve_direction,
                   std::stringstream* report)
{ return rdyn::saturateSpeed(chain,qd_target,max_velocity_multiplier,preserve_direction,report); }

[[deprecated("Include rdyn_core/kinematics_saturation.h and use namespace rdyn")]]
bool saturatePosition(const rdyn::Chain& chain,double& q_target, std::stringstream* report)
{ return rdyn::saturatePosition(chain,q_target,report); }

}  // namespace rdyn

#endif  // RDYN_CORE__KINEMATICS_SATURATION_H
