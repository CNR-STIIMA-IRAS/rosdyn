#ifndef frame_distance_201901161826
#define frame_distance_201901161826

#include <Eigen/Geometry>

namespace rosdyn
{

  /*
   * Distance between frames.
   * return a vector representing the distance of frame b w.r.t. to frame a expressed in frame w
   * the distance are defined as a vector containing:
   *   - the distance between origins (namely, origin_b_in_w - origin_a_in_w)
   *   - the product between the angle and unit vector  (AngleAxis)  expressed in frame w
   */
  inline void getFrameDistance(const Eigen::Affine3d& T_wa,  const Eigen::Affine3d& T_wb,  Eigen::VectorXd& distance)
  {
    distance.resize(6);
    distance.block(0, 0, 3, 1) = T_wa.translation()-T_wb.translation();
    Eigen::AngleAxisd aa_ab(T_wa.linear().inverse() * T_wb.linear());

      
    distance.block(3, 0, 3, 1) = -T_wa.linear()*(aa_ab.angle()*aa_ab.axis());
  }
  
  /*
   * Distance between frames.
   * return a vector representing the distance of frame b w.r.t. to frame a expressed in frame w
   * the distance are defined as a vector containing:
   *   - the distance between origins (namely, origin_b_in_w - origin_a_in_w)
   *   - the double of vector part of the quaternion q_ab expressed in frame w (namely R_wa * 2*imag(q_ab) ~= angle*axis)
   */
  inline void getFrameDistanceQuat(const Eigen::Affine3d& T_wa,  const Eigen::Affine3d& T_wb,  Eigen::VectorXd& distance)
  {
    distance.resize(6);
    distance.block(0, 0, 3, 1) = T_wa.translation()-T_wb.translation();
    Eigen::Quaterniond q_ab(T_wa.linear().inverse() * T_wb.linear());

    if (q_ab.w() <0)
    {
      q_ab.x()=-q_ab.x();
      q_ab.y()=-q_ab.y();
      q_ab.z()=-q_ab.z();
      q_ab.w()=-q_ab.w();
    }
      
    distance.block(3, 0, 3, 1) = -2.0*T_wa.linear() * q_ab.vec();
  }
  
  /*
   * Distance between frames and its jacobian.
   * return a vector representing the distance of frame b w.r.t. to frame a expressed in frame w
   * the distance are defined as a vector containing:
   *   - the distance between origins (namely, origin_b_in_w - origin_a_in_w)
   *   - the double of vector part of the quaternion q_ab expressed in frame w (namely R_wa * 2*imag(q_ab) ~= angle*axis)
   * and the jacobian of the distance error defined as
   * J=[eye(3) zeros(3, 3);zeros(3, 3) R_wa*(eye(3)*real(q_ab)-skew(imag(q_ab)))*R_wa']
   */
  inline void getFrameDistanceQuatJac(const Eigen::Affine3d& T_wa,  const Eigen::Affine3d& T_wb,  Eigen::VectorXd& distance,  Eigen::MatrixXd& jacobian)
  {
    distance.resize(6);
    jacobian.resize(6, 6);
    jacobian.setIdentity();
    
    distance.block(0, 0, 3, 1) = T_wb.translation()-T_wa.translation();
    Eigen::Quaterniond q_ab(T_wa.linear().inverse() * T_wb.linear());
    
    if (q_ab.w() <0)
    {
      q_ab.w()=-q_ab.w();
      q_ab.vec()=-q_ab.vec();
    }
      
      
    distance.block(3, 0, 3, 1) = -2.0*T_wa.linear() * q_ab.vec();
    
    jacobian.block(3, 3, 3, 3) = T_wa.linear() *( q_ab.w() *Eigen::MatrixXd::Identity(3, 3) - skew(q_ab.vec()) ) *T_wa.linear().inverse(); 
  }
  
  
}
#endif
