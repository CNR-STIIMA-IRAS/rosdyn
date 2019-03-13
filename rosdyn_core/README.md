![](../Documentation/rosdyn_logo.png)


ROSdyn_core is an Eigen-based Dynamics (header) library for robot chains. 


## Build/Installation 

See the instruction of the [main page](../README.md)

## List of packages

### **rosdyn_core**:
Dynamics header library based on Eigen. With respect to KDL, it has two advantages: it is faster and it allows to compute model regressor. 

An example of usage can be found [here](rosdyn_core/test/rosdyn_speed_test.cpp)


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

// jacobian of tool_frame in base_frame
Eigen::Matrix6Xd jacobian_of_tool_in_base;
jacobian_of_tool_in_base.resize(6, chain->getActiveJointsNumber());
jacobian_of_tool_in_base = chain->getJacobian(q);


// joint torque
Eigen::VectorXd tau(n_joints);
tau = chain->getJointTorque(q, Dq, DDq);


// nonlinear joint torque
Eigen::VectorXd nonlinear_tau(n_joints);
nonlinear_tau=chain->getJointTorqueNonLinearPart(q,Dq);


// joint inertia (tau = joint_inertia*DDq+ tau_non_linear)
Eigen::MatrixXd joint_inertia;
joint_inertia = chain->getJointInertia(q);


// twist_of_tool_in_base: velocity twist of tool frame w.r.t. base_frame
// nonlinacc_twist_of_tool_in_base: non linear part of the acceleration twist of tool frame w.r.t. base_frame
// linacc_twist_of_tool_in_base: linear part of the acceleration twist of tool frame w.r.t. base_frame
// acc_twist_of_tool_in_base: acceleration twist of tool frame w.r.t. base_frame
// yerk_twist_of_tool_in_base: yerk twist of tool frame w.r.t. base_frame

Eigen::Vector6d twist_of_tool_in_base;
Eigen::Vector6d nonlinacc_twist_of_tool_in_base;
Eigen::Vector6d linacc_twist_of_tool_in_base;
Eigen::Vector6d acc_twist_of_tool_in_base;
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
nonlinacc_twists = chain->getDTwistNonLinearPart(q, Dq);
acc_twists = chain->getDTwist(q, Dq, DDq);
jerk_twists = chain->getDDTwist(q, Dq, DDq, DDDq);


```
