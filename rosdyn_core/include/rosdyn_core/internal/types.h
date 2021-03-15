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
#ifndef ROSDYM_CORE__TYPES_H
#define ROSDYM_CORE__TYPES_H

#include <ros/common.h>
#include <urdf/model.h>
#include <urdf_model/model.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <rosdyn_core/spacevect_algebra.h>

#if ROS_VERSION_MINIMUM(1, 14, 1)
  #include <memory>
  namespace shared_ptr_namespace = std;
#else
  #include <boost/concept_check.hpp>
  #include <boost/graph/graph_concepts.hpp>
  #include <boost/enable_shared_from_this.hpp>
namespace shared_ptr_namespace = boost;
#endif

namespace urdf
{
#if defined(USE_RAW_POINTERS)
  typedef ::urdf::Joint* JointPtr;
  typedef ::urdf::Link*  LinkPtr;
  typedef const ::urdf::Joint* JointConstPtr;
  typedef const ::urdf::Link*  LinkConstPtr;
#else
  typedef shared_ptr_namespace::shared_ptr< ::urdf::Joint      > JointPtr;
  typedef shared_ptr_namespace::shared_ptr< ::urdf::Link       > LinkPtr;
  typedef shared_ptr_namespace::shared_ptr< ::urdf::Joint const> JointConstPtr;
  typedef shared_ptr_namespace::shared_ptr< ::urdf::Link  const> LinkConstPtr;
#endif
}

#if defined(USE_RAW_POINTERS)
#define NEW_HEAP(X, OBJECT)\
  X = new OBJECT;
#else
#define NEW_HEAP(X, OBJECT)\
  X.reset(new OBJECT);
#endif

#if defined(USE_RAW_POINTERS)
#define RESET_HEAP(X)
#else
#define RESET_HEAP(X)\
  X.reset();
#endif

#if defined(USE_RAW_POINTERS)
#define DELETE_HEAP(X)\
  delete X
#else
#define DELETE_HEAP(X)\
  X.reset();
#endif


#if defined(USE_RAW_POINTERS)
#define GET(X)\
  X.get()
#else
#define GET(X)\
  X
#endif


namespace rosdyn
{
class Joint;
class Link;
class Chain;

#if defined(USE_RAW_POINTERS)
typedef rosdyn::Joint* JointPtr;
typedef rosdyn::Link * LinkPtr;
typedef rosdyn::Chain* ChainPtr;

typedef const rosdyn::Joint* JointConstPtr;
typedef const rosdyn::Link * LinkConstPtr;
typedef const rosdyn::Chain* ChainConstPtr;
#else
typedef shared_ptr_namespace::shared_ptr< rosdyn::Joint   > JointPtr;
typedef shared_ptr_namespace::shared_ptr< rosdyn::Link    > LinkPtr;
typedef shared_ptr_namespace::shared_ptr< rosdyn::Chain   > ChainPtr;

typedef shared_ptr_namespace::shared_ptr< rosdyn::Joint const> JointConstPtr;
typedef shared_ptr_namespace::shared_ptr< rosdyn::Link  const> LinkConstPtr;
typedef shared_ptr_namespace::shared_ptr< rosdyn::Chain const> ChainConstPtr;
#endif

#if defined(MAX_NUM_AXES) && (MAX_NUM_AXES!=0)
  #define STR_HELPER(x) #x
  #define STR(x) STR_HELPER(x)
  #pragma message "ROSDYN MAX NUM AXES: " STR(MAX_NUM_AXES)
  #define NUM_MAX_AXES MAX_NUM_AXES
#else
  #define NUM_MAX_AXES 40
#endif

constexpr int max_num_axes = NUM_MAX_AXES;
constexpr int max_num_extended_axes = (NUM_MAX_AXES > 0 ? NUM_MAX_AXES * 10 : -1);

typedef Eigen::Matrix<double,-1, 1,Eigen::ColMajor,max_num_axes>                        VectorXd;
typedef Eigen::Matrix<double,-1,-1,Eigen::ColMajor,max_num_axes,max_num_axes>           MatrixXd;
typedef Eigen::Matrix<double, 6,-1,Eigen::ColMajor,           6,max_num_axes>           Matrix6Xd;
typedef Eigen::Matrix<double, 6, 6>                                                     Matrix66d;
typedef Eigen::Matrix<double,-1,-1,Eigen::ColMajor,max_num_axes,max_num_extended_axes>  ExtendedMatrixXd;
typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>         VectorOfAffine3d;
typedef std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>>         VectorOfVector6d;
typedef std::vector< Matrix66d, Eigen::aligned_allocator< Matrix66d > >                 VectorOfMatrix66d;
typedef Eigen::Matrix<double, 6, 10>                                                    Matrix610d;
typedef std::vector< Matrix610d, Eigen::aligned_allocator< Matrix610d > >               VectorOfMatrix610d;

}

#endif

