#ifndef ROSDYN_CORE__KINEMATICS_SATURATION_IMPL__H
#define ROSDYN_CORE__KINEMATICS_SATURATION_IMPL__H

#include <Eigen/Dense>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <kinematics_filters/kinematics_filters.h>
#include <rosdyn_core/kinematics_saturation.h>

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

  return cnr_control_toolbox::saturateSpeed(qd_next, chain.getDQMax(), max_velocity_multiplier, preserve_direction, report);
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

    return  cnr_control_toolbox::saturateSpeed(qd_next, 
        qd_actual, chain.getDQMax(),chain.getDDQMax(),dt,max_velocity_multiplier,preserve_direction,report);
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
    return cnr_control_toolbox::saturateSpeed(qd_next,
        q_actual, qd_actual, chain.getQMax(), chain.getQMin(), chain.getDQMax(), chain.getDDQMax(),
            dt,max_velocity_multiplier,preserve_direction,report);
}

template<typename D1>
inline bool saturatePosition(const rosdyn::Chain& chain, Eigen::MatrixBase<D1>& q_next, std::stringstream* report)
{
    return cnr_control_toolbox::saturatePosition(q_next, chain.getQMax(), chain.getQMin(), report);
}





inline bool saturateSpeed(const rosdyn::Chain& chain, double& qd_next, double max_velocity_multiplier,
                            bool preserve_direction, std::stringstream* report)
{
    return cnr_control_toolbox::saturateSpeed(qd_next, chain.getDQMax(0), max_velocity_multiplier, preserve_direction, report);
}


inline bool saturateSpeed(const rosdyn::Chain& chain, double& qd_next, const double& qd_actual, double dt,
                            double max_velocity_multiplier, bool preserve_direction, std::stringstream* report)
{
    return  cnr_control_toolbox::saturateSpeed(qd_next, 
        qd_actual, chain.getDQMax(0),chain.getDDQMax(0),dt,max_velocity_multiplier,preserve_direction,report);
}


inline bool saturateSpeed(const rosdyn::Chain& chain, double& qd_next, const double& qd_actual, const double& q_actual,
                          double dt, double max_velocity_multiplier, bool preserve_direction, std::stringstream* report)
{
    return cnr_control_toolbox::saturateSpeed(qd_next,
    q_actual, qd_actual, chain.getQMax(0), chain.getQMin(0), chain.getDQMax(0), chain.getDDQMax(0),
        dt,max_velocity_multiplier,preserve_direction,report);
}


inline bool saturatePosition(const rosdyn::Chain& chain, double& q_next, std::stringstream* report)
{
    return cnr_control_toolbox::saturatePosition(q_next, chain.getQMax(0), chain.getQMin(0), report);
}

}  // namespace rosdyn


#endif  // ROSDYN_CORE__KINEMATICS_SATURATION_IMPL__H