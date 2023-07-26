/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once  // NOLINT(build/header_guard)
#include <string>
#include <vector>

#include <rdyn_core/frame_distance.h>

namespace rosdyn
{

/*
 * Distance between frames.
 * return a vector representing the distance of frame b w.r.t. to frame a expressed in frame w
 * the distance are defined as a vector containing:
 *   - the distance between origins (namely, origin_b_in_w - origin_a_in_w)
 *   - the product between the angle and unit vector  (AngleAxis)  expressed in frame w
 */
[[deprecated("Include rdyn_core/kinematics_saturation.h and use namespace rdyn")]]
inline void getFrameDistance(const Eigen::Affine3d& T_wa,  const Eigen::Affine3d& T_wb,  Eigen::Matrix<double,6,1>& distance)
{ return rdyn::getFrameDistance(T_wa,T_wb,distance); }

/*
 * Distance between frames.
 * return a vector representing the distance of frame b w.r.t. to frame a expressed in frame w
 * the distance are defined as a vector containing:
 *   - the distance between origins (namely, origin_b_in_w - origin_a_in_w)
 *   - the product between the angle and unit vector  (AngleAxis)  expressed in frame w
 */
[[deprecated("Use the getFrameDistance(const Eigen::Affine3d&,const Eigen::Affine3d&,Eigen::Matrix<double,6,1>&")]]
inline void getFrameDistance(const Eigen::Affine3d& T_wa,  const Eigen::Affine3d& T_wb,  Eigen::VectorXd& distance)
{ return rdyn::getFrameDistance(T_wa,T_wb,distance); }

/*
 * Distance between frames.
 * return a vector representing the distance of frame b w.r.t. to frame a expressed in frame w
 * the distance are defined as a vector containing:
 *   - the distance between origins (namely, origin_b_in_w - origin_a_in_w)
 *   - the double of vector part of the quaternion q_ab expressed in frame w (namely R_wa * 2*imag(q_ab) ~= angle*axis)
 */
[[deprecated("Include rdyn_core/kinematics_saturation.h and use namespace rdyn")]]
inline void getFrameDistanceQuat(const Eigen::Affine3d& T_wa,  const Eigen::Affine3d& T_wb, Eigen::Matrix<double,6,1>& distance)
{ return rdyn::getFrameDistanceQuat(T_wa,  T_wb, distance); }

/*
 * Distance between frames.
 * return a vector representing the distance of frame b w.r.t. to frame a expressed in frame w
 * the distance are defined as a vector containing:
 *   - the distance between origins (namely, origin_b_in_w - origin_a_in_w)
 *   - the double of vector part of the quaternion q_ab expressed in frame w (namely R_wa * 2*imag(q_ab) ~= angle*axis)
 */
[[deprecated("Use the getFrameDistanceQuat(const Eigen::Affine3d&,const Eigen::Affine3d&,Eigen::Matrix<double,6,1>&")]]
inline void getFrameDistanceQuat(const Eigen::Affine3d& T_wa,  const Eigen::Affine3d& T_wb,  Eigen::VectorXd& distance)
{return rdyn::getFrameDistanceQuat(T_wa,  T_wb,  distance); }

/*
 * Distance between frames and its jacobian.
 * return a vector representing the distance of frame b w.r.t. to frame a expressed in frame w
 * the distance are defined as a vector containing:
 *   - the distance between origins (namely, origin_b_in_w - origin_a_in_w)
 *   - the double of vector part of the quaternion q_ab expressed in frame w (namely R_wa * 2*imag(q_ab) ~= angle*axis)
 * and the jacobian of the distance error defined as
 * J=[eye(3) zeros(3, 3);zeros(3, 3) R_wa*(eye(3)*real(q_ab)-skew(imag(q_ab)))*R_wa']
 */
[[deprecated("Include rdyn_core/kinematics_saturation.h and use namespace rdyn")]]
inline void getFrameDistanceQuatJac(const Eigen::Affine3d& T_wa,  const Eigen::Affine3d& T_wb,  Eigen::Matrix<double,6,1>& distance, Eigen::Matrix<double,6,6>& jacobian)
{return rdyn::getFrameDistanceQuatJac(T_wa,T_wb,distance,jacobian);}
/*
 * Distance between frames and its jacobian.
 * return a vector representing the distance of frame b w.r.t. to frame a expressed in frame w
 * the distance are defined as a vector containing:
 *   - the distance between origins (namely, origin_b_in_w - origin_a_in_w)
 *   - the double of vector part of the quaternion q_ab expressed in frame w (namely R_wa * 2*imag(q_ab) ~= angle*axis)
 * and the jacobian of the distance error defined as
 * J=[eye(3) zeros(3, 3);zeros(3, 3) R_wa*(eye(3)*real(q_ab)-skew(imag(q_ab)))*R_wa']
 */
[[deprecated("Use the getFrameDistanceQuatJac(const Eigen::Affine3d&,const Eigen::Affine3d&,Eigen::Matrix<double,6,1>&,Eigen::Matrix<double,6,6>&")]]
inline void getFrameDistanceQuatJac(const Eigen::Affine3d& T_wa,  const Eigen::Affine3d& T_wb,  Eigen::VectorXd& distance,  Eigen::MatrixXd& jacobian)
{  return rdyn::getFrameDistanceQuatJac(T_wa, T_wb,  distance, jacobian);}

}  // namespace rdyn

