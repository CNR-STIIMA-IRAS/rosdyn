#ifndef __ITIA_KIN_DERIVATIVES__
#define __ITIA_KIN_DERIVATIVES__

# include <urdf/model.h>
# include <ros/console.h>
# include <Eigen/Geometry>
# include <Eigen/StdVector>
# include <boost/enable_shared_from_this.hpp>
# include <boost/shared_ptr.hpp>
# include <rosdyn_core/spacevect_algebra.h>


namespace rosdyn
{
  
  inline Eigen::Affine3d urdfPoseToAffine(urdf::Pose pose)
  {
    Eigen::Affine3d affine;
    affine = Eigen::Quaterniond(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
    affine.translation() << pose.position.x, pose.position.y, pose.position.z;
    return affine;  
  }
  
  inline Eigen::Vector3d urdfVectorToEigen(urdf::Vector3 vector)
  {
    Eigen::Vector3d eigen_vector;
    eigen_vector  << vector.x, vector.y, vector.z;
    return eigen_vector;
  }

}

# endif