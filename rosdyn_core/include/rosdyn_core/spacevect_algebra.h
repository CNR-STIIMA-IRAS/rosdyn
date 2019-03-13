#ifndef ITIA_SPACEVECT_ALGEBRA__201806200756
#define ITIA_SPACEVECT_ALGEBRA__201806200756

# include <Eigen/Geometry>
# include <Eigen/Core>

namespace Eigen
{
  typedef Matrix<double, 6,  1> Vector6d;
  typedef Matrix<double, 6, -1> Matrix6Xd;
  typedef Matrix<double, 6, 6>  Matrix66d;
}



/* Conventions
 *
 * twist = [
 * translation velocity
 * angular velocity ]
 *
 * wrench = [
 * force
 * torque ]
 *
 * twist_of_a_in_b = twist representing the translation and angular velocity of the origin of frame a ( point of application ), expressed in frame b (reference frame)
 * wrench_of_a_in_b = wrench representing the force and torque  applied in the origin of frame a ( point of application ), expressed in frame b (reference frame)
 *
 * spatial operator is described in https://www.springer.com/gp/book/9781475764376 (note that in the book, twists and wrenches have the angular part before the linear one)
 *
 * for skew, visit https://en.wikipedia.org/wiki/Skew-symmetric_matrix
 *
 *
 */

namespace rosdyn
{

/* compute the skew-symmetric matrix of a vector
 */
inline Eigen::Matrix3d skew(const Eigen::Vector3d& vec)
{
  Eigen::Matrix3d mat;
  mat << 0, -vec(2), vec(1),
    vec(2), 0, -vec(0),
    -vec(1), vec(0), 0;
  return mat;
}

/* compute the vector from a skew-symmetric matrix
 */
inline Eigen::Vector3d unskew(const Eigen::Matrix3d& mat)
{
  return Eigen::Vector3d(mat(2, 1), mat(0, 2), mat(1, 0));
}

/*
 * for twist
 */
inline void spatialCrossProduct(const Eigen::Vector6d& vet1, const Eigen::Vector6d& vet2, Eigen::Vector6d* res)
{
  res->block(3, 0, 3, 1) = ((Eigen::Vector3d)(vet1.block(3, 0, 3, 1))).cross((Eigen::Vector3d)vet2.block(3, 0, 3, 1));
  res->block(0, 0, 3, 1) = ((Eigen::Vector3d)(vet1.block(3, 0, 3, 1))).cross((Eigen::Vector3d)vet2.block(0, 0, 3, 1)) +
                           ((Eigen::Vector3d)(vet1.block(0, 0, 3, 1))).cross((Eigen::Vector3d)vet2.block(3, 0, 3, 1));
}

/*
 * for twist
 */
inline Eigen::Vector6d spatialCrossProduct(const Eigen::Vector6d& vet1, const Eigen::Vector6d& vet2)
{
  Eigen::Vector6d res;
  spatialCrossProduct(vet1, vet2, &res);
  return res;
}

/*
 * for wrench
 */
inline void spatialDualCrossProduct(const Eigen::Vector6d& vet1, const Eigen::Vector6d& vet2, Eigen::Vector6d* res)
{
  res->block(3, 0, 3, 1) = ((Eigen::Vector3d)(vet1.block(3, 0, 3, 1))).cross((Eigen::Vector3d)vet2.block(3, 0, 3, 1))+
                           ((Eigen::Vector3d)(vet1.block(0, 0, 3, 1))).cross((Eigen::Vector3d)vet2.block(0, 0, 3, 1));
  res->block(0, 0, 3, 1) = ((Eigen::Vector3d)(vet1.block(3, 0, 3, 1))).cross((Eigen::Vector3d)vet2.block(0, 0, 3, 1));
}

/*
 * for wrench
 */
inline Eigen::Vector6d spatialDualCrossProduct(const Eigen::Vector6d& vet1, const Eigen::Vector6d& vet2)
{
  Eigen::Vector6d res;
  spatialDualCrossProduct(vet1, vet2, &res);
  return res;
}

/*
 * TWIST: Change point of application from a to b without changing thereference frame b.
 * translate the twist_of_a_in_b to the twist_of_c_in_b, where distance_from_a_to_c_in_b is the distance between the frames expressed in frame b
 */
inline void spatialTranslation(const Eigen::Vector6d& twist_of_a_in_b, const Eigen::Vector3d& distance_from_a_to_c_in_b, Eigen::Vector6d* twist_of_c_in_b)
{
 (*twist_of_c_in_b) = twist_of_a_in_b;
 twist_of_c_in_b->block(0, 0, 3, 1) = twist_of_a_in_b.block(0, 0, 3, 1) + ((Eigen::Vector3d)(twist_of_a_in_b.block(3, 0, 3, 1))).cross(distance_from_a_to_c_in_b);
}

/*
 * TWIST: Change point of application from a to b without changing thereference frame b.
 * translate the twist_of_a_in_b to the twist_of_c_in_b, where distance_from_a_to_c_in_b is the distance between the frames expressed in frame b
 */
inline Eigen::Vector6d spatialTranslation(const Eigen::Vector6d& twist_of_a_in_b, const Eigen::Vector3d& distance_from_a_to_c_in_b)
{
  Eigen::Vector6d twist_of_c_in_b;
  spatialTranslation(twist_of_a_in_b, distance_from_a_to_c_in_b, &twist_of_c_in_b);
  return twist_of_c_in_b;
}

/*
 * WRENCH: Change point of application from a to b without changing thereference frame b.
 * translate the wrench_of_a_in_b to the wrench_of_c_in_b, where distance_from_a_to_c_in_b is the distance between the frames expressed in frame b
 */
inline void spatialDualTranslation(const Eigen::Vector6d& wrench_of_a_in_b, const Eigen::Vector3d& distance_from_a_to_c_in_b, Eigen::Vector6d* wrench_of_c_in_b)
{
 (*wrench_of_c_in_b) = wrench_of_a_in_b;
 wrench_of_c_in_b->block(3, 0, 3, 1) = wrench_of_a_in_b.block(3, 0, 3, 1) + ((Eigen::Vector3d)(wrench_of_a_in_b.block(0, 0, 3, 1))).cross(distance_from_a_to_c_in_b);
}

/*
 * WRENCH: Change point of application from a to b without changing thereference frame b.
 * translate the wrench_of_a_in_b to the wrench_of_c_in_b, where distance_from_a_to_c_in_b is the distance between the frames expressed in frame b
 */
inline Eigen::Vector6d spatialDualTranslation(const Eigen::Vector6d& wrench_of_a_in_b, const Eigen::Vector3d& distance_from_a_to_c_in_b)
{
  Eigen::Vector6d wrench_of_c_in_b;
  spatialDualTranslation(wrench_of_a_in_b, distance_from_a_to_c_in_b, &wrench_of_c_in_b);
  return wrench_of_c_in_b;
}

/*
 * TWIST AND WRENCH: change the reference frame without change the  point of application.
 * Rotate twist_of_a_in_b to twist_of_a_c, applying the rotation rot_b_c (= T_b_c.linear())
 * Rotate wrench_of_a_in_b to wrench_of_a_c, applying the rotation rot_b_c (= T_b_c.linear())
 */
inline void spatialRotation(const Eigen::Vector6d& vec6_of_a_in_b, const Eigen::Matrix3d& rot_b_c, Eigen::Vector6d* vec6_of_a_in_c)
{
  (*vec6_of_a_in_c) << rot_b_c*vec6_of_a_in_b.block(0, 0, 3, 1), rot_b_c*vec6_of_a_in_b.block(3, 0, 3, 1);
}

/*
 * TWIST AND WRENCH: change the reference frame without change the  point of application.
 * Rotate twist_of_a_in_b to twist_of_a_c, applying the rotation rot_b_c (= T_b_c.linear())
 * Rotate wrench_of_a_in_b to wrench_of_a_c, applying the rotation rot_b_c (= T_b_c.linear())
 */
inline Eigen::Vector6d spatialRotation(const Eigen::Vector6d& vec6_of_a_in_b, const Eigen::Matrix3d& rot_b_c)
{
  Eigen::Vector6d vec6_of_a_in_c;
  spatialRotation(vec6_of_a_in_b, rot_b_c, &vec6_of_a_in_c);
  return vec6_of_a_in_c;
}

/*
 * TWIST: change the reference frame and change the  point of application.
 * Rototraslation twist_of_a_in_a to twist_of_b_b, applying the transformation T_b_c
 */
inline void spatialTranformation(const Eigen::Vector6d& twist_of_a_in_a, const Eigen::Affine3d& T_b_a, Eigen::Vector6d* twist_of_b_in_b)
{
  (*twist_of_b_in_b) << T_b_a.linear()*twist_of_a_in_a.block(0, 0, 3, 1) + ((Eigen::Vector3d)(twist_of_a_in_a.block(3, 0, 3, 1))).cross(T_b_a.translation()),
            T_b_a.linear()*twist_of_a_in_a.block(3, 0, 3, 1);
}

/*
 * TWIST: change the reference frame and change the  point of application.
 * Rototraslation twist_of_a_in_a to twist_of_b_b, applying the transformation T_b_a
 */
inline Eigen::Vector6d spatialTranformation(const Eigen::Vector6d& twist_of_a_in_a, const Eigen::Affine3d& T_b_a)
{
  Eigen::Vector6d twist_of_b_in_b;
  spatialTranformation(twist_of_a_in_a, T_b_a, &twist_of_b_in_b);
  return twist_of_b_in_b;
}

/*
 * WRENCH: change the reference frame and change the  point of application.
 * Rototraslation wrench_of_a_in_a to wrench_of_b_b, applying the transformation T_b_a
 */
inline void spatialDualTranformation(const Eigen::Vector6d& wrench_of_a_in_a, const Eigen::Affine3d& T_b_a, Eigen::Vector6d* wrench_of_b_b)
{
  (*wrench_of_b_b) << T_b_a.linear()*wrench_of_a_in_a.block(0, 0, 3, 1),
            T_b_a.linear()*wrench_of_a_in_a.block(3, 0, 3, 1) + ((Eigen::Vector3d)(wrench_of_a_in_a.block(0, 0, 3, 1))).cross(T_b_a.translation());
}

/*
 * WRENCH: change the reference frame and change the  point of application.
 * Rototraslation wrench_of_a_in_a to wrench_of_b_b, applying the transformation T_b_a
 */
inline Eigen::Vector6d spatialDualTranformation(const Eigen::Vector6d& wrench_of_a_in_a, const Eigen::Affine3d& T_b_a)
{
  Eigen::Vector6d wrench_of_b_b;
  spatialDualTranformation(wrench_of_a_in_a, T_b_a, &wrench_of_b_b);
  return wrench_of_b_b;
}


inline void computeSpatialInertiaMatrix(const Eigen::Ref<Eigen::Matrix3d>& inertia, const Eigen::Ref<Eigen::Vector3d> cog, const double& mass, Eigen::Ref<Eigen::Matrix<double, 6, 6>> spatial_inertia)
{
  Eigen::Matrix3d cog_skew = rosdyn::skew(cog);
  spatial_inertia.block(0, 0, 3, 3) = mass*Eigen::MatrixXd::Identity(3, 3);
  spatial_inertia.block(0, 3, 3, 3) = mass*cog_skew.transpose();
  spatial_inertia.block(3, 0, 3, 3) = mass*cog_skew;
  spatial_inertia.block(3, 3, 3, 3) = inertia + mass*(cog_skew*cog_skew.transpose());
  
}


}


# endif 
