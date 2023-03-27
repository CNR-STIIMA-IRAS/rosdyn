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

#ifndef ROSDYN_CORE_PRIMITIVES_H
#define ROSDYN_CORE_PRIMITIVES_H

#include <string>
#include <vector>
#include <Eigen/Core>

# include <rdyn_core/internal/types.h>
# include <rdyn_core/primitives.h>

namespace rosdyn
{

using Joint = rdyn::Joint;
using JointPtr = rdyn::JointPtr;
using JointConstPtr = rdyn::JointConstPtr;
using Link = rdyn::Link;
using LinkPtr = rdyn::LinkPtr;
using LinkConstPtr = rdyn::LinkConstPtr;


using VectorXd = rdyn::VectorXd;
using MatrixXd = rdyn::MatrixXd;
using Matrix6Xd = rdyn::Matrix6Xd;
using Matrix66d = rdyn::Matrix66d;
using ExtendedMatrixXd = rdyn::ExtendedMatrixXd;
using VectorOfAffine3d = rdyn::VectorOfAffine3d;
using VectorOfVector6d = rdyn::VectorOfVector6d;
using VectorOfMatrix66d = rdyn::VectorOfMatrix66d;
using Matrix610d = rdyn::Matrix610d;
using VectorOfMatrix610d = rdyn::VectorOfMatrix610d;

/**
 * 
 */
class Chain : public rdyn::Chain
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  #if defined(USE_RAW_POINTERS)
    using Ptr = Chain*;
    using ConstPtr = const Chain*;
  #else
    using Ptr = shared_ptr_namespace::shared_ptr< rosdyn::Chain >;
    using ConstPtr = shared_ptr_namespace::shared_ptr< rosdyn::Chain const>;
  #endif


  Chain() = default;
  ~Chain() = default;
  Chain(const Chain&);
  Chain(Chain&&) = delete;
  Chain& operator=(const Chain&);
  Chain& operator=(Chain&&) = delete;

  Chain(const rosdyn::LinkPtr& root_link,
          const std::string& base_link_name,
            const std::string& ee_link_name,
              const Eigen::Vector3d& gravity = Eigen::Vector3d::Zero());
  Chain(const urdf::Model& model,
          const std::string& base_link_name,
            const std::string& ee_link_name, 
              const Eigen::Vector3d& gravity = Eigen::Vector3d::Zero());

  /**
   * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   * AS DIFFERENCE FROM rdyn::Chain, the first param is the ros parameter 
   * namespace where the urdf is stored, and not the file path. 
   * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  */
  Chain(const std::string& robot_description,
          const std::string& base_link_name,
            const std::string& ee_link_name,
              const Eigen::Vector3d& gravity = Eigen::Vector3d::Zero());

  int  enforceLimitsFromRobotDescriptionParam(const std::string& full_param_path, std::string& error);
  bool setInputJointsName(const std::vector<std::string>& joints_name);
};

using ChainPtr = ::rosdyn::Chain::Ptr;
using ConstChainPtr = ::rosdyn::Chain::ConstPtr;

/**
 * @brief construct the shared_ptr of a chain
 * 
 * @param[in] urdf
 * @param[in] base frame
 * @param[in] tool frame
 * @param[in] gravity vector
 */
rosdyn::ChainPtr createChain( const urdf::ModelInterface& urdf_model_interface,
                              const std::string& base_frame,
                              const std::string& tool_frame, 
                              const Eigen::Vector3d& gravity);


/**
 * @brief construct the shared_ptr of a chain
 * 
 * @param[in] Chain
 */
rosdyn::ChainPtr createChain(const rosdyn::ChainPtr& chain);

/**
 * @brief construct the shared_ptr of a chain
 *
 * @param[in] Chain
 */
rosdyn::ChainPtr createChain(const rosdyn::Chain& chain);



///////////////////////////////////////////////////

}  // namespace rosdyn

#include <rosdyn_core/internal/primitives_impl.h>

#endif  // ROSDYN_CORE_PRIMITIVES_H 
