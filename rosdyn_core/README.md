![](../Documentation/rosdyn_logo.png)


ROSdyn_core is an Eigen-based Dynamics (header) library for robot chains. 


## Build/Installation 

See the instruction of the [main page](../README.md)

## List of packages

### **rosdyn_core**:
Dynamics header library based on Eigen. With respect to KDL, it has two advantages: it is faster and it allows computing model regressor. 

An example of usage can be found [here](test/rosdyn_speed_test.cpp)


## Usage

You need to include:
```c++
#include <rosdyn_core/primitives.h>
#include <rosdyn_core/urdf_parser.h>
```

Creating a chain
```c++
  urdf::Model model;
  model.initParam("robot_description");
  
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;
  
  boost::shared_ptr<rosdyn::Chain> chain = rosdyn::createChain(model, base_frame,tool_frame,grav);
```

Initializing variables

```c++

unsigned int n_joints=chain->getActiveJointsNumber();

//joint positions
Eigen::VectorXd q(n_joints);
q.setZero();

//joint velocity
Eigen::VectorXd Dq(n_joints);
Dq.setZero();

//joint acceleration
Eigen::VectorXd DDq(n_joints);
DDq.setZero();

// joint jerk
Eigen::VectorXd DDDq(n_joints);
DDDq.setZero();


```

Main methods:
```c++  
// transform base_frame <- tool_frame 
Eigen::Affine3d T_base_tool;
T_base_tool = chain->getTransformation(q);

// get the transforms for all the links
std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> T_base_all_links;
T_base_all_links = chain->getTransformations(q);

// jacobian of tool_frame in base_frame
Eigen::Matrix6Xd jacobian_of_tool_in_base;
jacobian_of_tool_in_base.resize(6, chain->getActiveJointsNumber());
jacobian_of_tool_in_base = chain->getJacobian(q);

// inverse kinematics
Eigen::Affine3d T_base_tool; // desired transformation
Eigen::VectorXd seed; // initial seed
Eigen::VectorXd sol;  // solution
if (chian->computeLocalIk(sol, T_base_tool, seed))
  ROS_INFO("ok");
else
  ROS_ERROR("no solution found");


// joint torque tau = joint_inertia_matrix*DDq+ tau_non_linear
Eigen::VectorXd tau(n_joints);
tau = chain->getJointTorque(q, Dq, DDq);


Eigen::MatrixXd joint_inertia_matrix;
joint_inertia_matrix = chain->getJointInertia(q);

// nonlinear joint torque (gravitational+Coriolis)
Eigen::VectorXd nonlinear_tau(n_joints);
nonlinear_tau=chain->getJointTorqueNonLinearPart(q,Dq);

Eigen::VectorXd gravitational_tau(n_joints);
gravitational_tau=chain->getJointTorqueNonLinearPart(q,0*Dq);




// twist_of_tool_in_base: velocity twist of tool frame w.r.t. base_frame
// acc_twist_of_tool_in_base: acceleration twist of tool frame w.r.t. base_frame
// nonlinacc_twist_of_tool_in_base: non linear part of the acceleration twist of tool frame w.r.t. base_frame
// linacc_twist_of_tool_in_base: linear part of the acceleration twist of tool frame w.r.t. base_frame
// yerk_twist_of_tool_in_base: yerk twist of tool frame w.r.t. base_frame
//
// NOTE: twist_of_tool_in_base = [linear_velocity; angular_velocity]
// NOTE: acc_twist_of_tool_in_base = [linear_acceleration; angular_acceleration]


Eigen::Vector6d twist_of_tool_in_base;
Eigen::Vector6d acc_twist_of_tool_in_base;
Eigen::Vector6d nonlinacc_twist_of_tool_in_base;
Eigen::Vector6d linacc_twist_of_tool_in_base;
Eigen::Vector6d jerk_twist_of_tool_in_base;

twist_of_tool_in_base=chain->getTwistTool(q,Dq);
linacc_twist_of_tool_in_base = chain->getDTwistLinearPartTool(q, DDq);
nonlinacc_twist_of_tool_in_base = chain->getDTwistNonLinearPartTool(q, Dq);
acc_twist_of_tool_in_base = chain->getDTwistTool(q, Dq, DDq);
jerk_twist_of_tool_in_base = chain->getDDTwistTool(q, Dq, DDq, DDDq);




// twists: velocity twists of all the links w.r.t. base_frame
// nonlinacc_twists: non linear part of the acceleration twists of all the links w.r.t. base_frame
// linacc_twists: linear part of the acceleration twists of all the links w.r.t. base_frame
// acc_twists: acceleration twists of all the links w.r.t. base_frame
// yerk_twists: yerk twists of all the links w.r.t. base_frame

std::vector< Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d> > twists, nonlinacc_twists, linacc_twists, acc_twists, jerk_twists;
twists = chain->getTwist(q, Dq);
linacc_twists = chain->getDTwistLinearPart(q, DDq);
t_linacc_eigen +=  (ros::Time::now()-t0).toSec() * 1e6;
acc_twists = chain->getDTwist(q, Dq, DDq);
jerk_twists = chain->getDDTwist(q, Dq, DDq, DDDq);


```


## Spatial vector algebra

rosdyn_core provides functionalities to rotate and/or translate twists and wrenches.
spatial operator is described in ["Rigid Body Dynamics Algorithms", Roy Featherstone](https://www.springer.com/gp/book/9781475764376) (note that in the book, twists and wrenches have the angular part before the translational one).

**NOTE**: *Dual* operations are applied to wrench, only spatialRotation can be applied to twists and wrenches.

Notation:

> twist = [vx, vy, vz, wx, wy, wz]

>> vx, vy, vz  translational velocity

>> wx, wy, wz  angular velocity

> wrench = [Fx, Fy, Fz, Tx, Ty, Tz]

>> Fx, Fy, Fz force vector

>> Tx, Ty, Tz torque vector


 > twist_of_a_in_b = twist representing the translation and angular velocity of the origin of frame a ( point of application ), expressed in frame b (reference frame)
 
 > wrench_of_a_in_b = wrench representing the force and torque  applied in the origin of frame a ( point of application ), expressed in frame b (reference frame)

```c++

/* compute the skew-symmetric matrix of a vector
 */
inline Eigen::Matrix3d skew(const Eigen::Vector3d& vec);

/* compute the vector from a skew-symmetric matrix
 */
inline Eigen::Vector3d unskew(const Eigen::Matrix3d& mat);

/*
 * for twist
 */
inline void spatialCrossProduct(const Eigen::Vector6d& vet1, const Eigen::Vector6d& vet2, Eigen::Vector6d* res);

/*
 * for twist
 */
inline Eigen::Vector6d spatialCrossProduct(const Eigen::Vector6d& vet1, const Eigen::Vector6d& vet2);


/*
 * TWIST: Change point of application from a to b without changing thereference frame b.
 * translate the twist_of_a_in_b to the twist_of_c_in_b, where distance_from_a_to_c_in_b is the distance between the frames expressed in frame b
 */
inline void spatialTranslation(const Eigen::Vector6d& twist_of_a_in_b, const Eigen::Vector3d& distance_from_a_to_c_in_b, Eigen::Vector6d* twist_of_c_in_b);

/*
 * TWIST: Change point of application from a to b without changing thereference frame b.
 * translate the twist_of_a_in_b to the twist_of_c_in_b, where distance_from_a_to_c_in_b is the distance between the frames expressed in frame b
 */
inline Eigen::Vector6d spatialTranslation(const Eigen::Vector6d& twist_of_a_in_b, const Eigen::Vector3d& distance_from_a_to_c_in_b);

/*
 * TWIST: change the reference frame and change the  point of application.
 * Rototraslation twist_of_a_in_a to twist_of_b_b, applying the transformation T_b_c
 */
inline void spatialTranformation(const Eigen::Vector6d& twist_of_a_in_a, const Eigen::Affine3d& T_b_a, Eigen::Vector6d* twist_of_b_in_b);

/*
 * TWIST: change the reference frame and change the  point of application.
 * Rototraslation twist_of_a_in_a to twist_of_b_b, applying the transformation T_b_a
 */
inline Eigen::Vector6d spatialTranformation(const Eigen::Vector6d& twist_of_a_in_a, const Eigen::Affine3d& T_b_a);



/*
 * WRENCH: Change point of application from a to b without changing thereference frame b.
 * translate the wrench_of_a_in_b to the wrench_of_c_in_b, where distance_from_a_to_c_in_b is the distance between the frames expressed in frame b
 */
inline void spatialDualTranslation(const Eigen::Vector6d& wrench_of_a_in_b, const Eigen::Vector3d& distance_from_a_to_c_in_b, Eigen::Vector6d* wrench_of_c_in_b);

/*
 * WRENCH: Change point of application from a to b without changing thereference frame b.
 * translate the wrench_of_a_in_b to the wrench_of_c_in_b, where distance_from_a_to_c_in_b is the distance between the frames expressed in frame b
 */
inline Eigen::Vector6d spatialDualTranslation(const Eigen::Vector6d& wrench_of_a_in_b, const Eigen::Vector3d& distance_from_a_to_c_in_b);

/*
 * for wrench
 */
inline void spatialDualCrossProduct(const Eigen::Vector6d& vet1, const Eigen::Vector6d& vet2, Eigen::Vector6d* res);

/*
 * for wrench
 */
inline Eigen::Vector6d spatialDualCrossProduct(const Eigen::Vector6d& vet1, const Eigen::Vector6d& vet2);

/*
 * WRENCH: change the reference frame and change the  point of application.
 * Rototraslation wrench_of_a_in_a to wrench_of_b_b, applying the transformation T_b_a
 */
inline void spatialDualTranformation(const Eigen::Vector6d& wrench_of_a_in_a, const Eigen::Affine3d& T_b_a, Eigen::Vector6d* wrench_of_b_b);

/*
 * WRENCH: change the reference frame and change the  point of application.
 * Rototraslation wrench_of_a_in_a to wrench_of_b_b, applying the transformation T_b_a
 */
inline Eigen::Vector6d spatialDualTranformation(const Eigen::Vector6d& wrench_of_a_in_a, const Eigen::Affine3d& T_b_a);


/*
 * TWIST AND WRENCH: change the reference frame without change the  point of application.
 * Rotate twist_of_a_in_b to twist_of_a_c, applying the rotation rot_b_c (= T_b_c.linear())
 * Rotate wrench_of_a_in_b to wrench_of_a_c, applying the rotation rot_b_c (= T_b_c.linear())
 */
inline void spatialRotation(const Eigen::Vector6d& vec6_of_a_in_b, const Eigen::Matrix3d& rot_b_c, Eigen::Vector6d* vec6_of_a_in_c);

/*
 * TWIST AND WRENCH: change the reference frame without change the  point of application.
 * Rotate twist_of_a_in_b to twist_of_a_c, applying the rotation rot_b_c (= T_b_c.linear())
 * Rotate wrench_of_a_in_b to wrench_of_a_c, applying the rotation rot_b_c (= T_b_c.linear())
 */
inline Eigen::Vector6d spatialRotation(const Eigen::Vector6d& vec6_of_a_in_b, const Eigen::Matrix3d& rot_b_c);



inline void computeSpatialInertiaMatrix(const Eigen::Ref<Eigen::Matrix3d>& inertia, const Eigen::Ref<Eigen::Vector3d> cog, const double& mass, Eigen::Ref<Eigen::Matrix<double, 6, 6>> spatial_inertia);




```
