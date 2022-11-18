![](Documentation/rosdyn_logo.png)


ROSdyn implements a fully automated procedure able to calibrate the robot dynamics model.

It is integrated with MoveIt! to automatically compute, simulate, and execute identification trajectory. The result is stored in a URDF file.

## Build/Installation

The software can be installed with the following [rosinstall file](rosdyn.rosinstall).

Travis CI Kinetic Build: [![Build Status](https://travis-ci.org/CNR-STIIMA-IRAS/rosdyn.svg?branch=melodic-devel)](https://travis-ci.org/CNR-STIIMA-IRAS/rosdyn)


__IMPORTANT__ rosdyn_identification has been moved [here](https://github.com/CNR-STIIMA-IRAS/rosdyn_identification).

## List of packages

### **rosdyn_core** [see README](rosdyn_core/README.md):
Dynamics header library based on Eigen. With respect to KDL, it has two advantages: it is faster and it allow computing model regressor.


An example of usage can be found [here](rosdyn_core/test/rosdyn_speed_test.cpp)

The following list shows the computation times for a 6DOF robot on a laptop Asus PU551J with Ubuntu 16.04 (Release build,  average on 10000 trials).

** computation time in microseconds: **

> pose                                            =  0.75970 [us]

> pose + jacobian                                       =  1.06562 [us]

> pose + jacobian + velocity twists for all links                   =  1.25589 [us]

> pose + jacobian + velocity twists for all links + linear aceleration twists for all links        =  1.25351 [us]

> pose + jacobian + velocity twists for all links +  non linear acceleration twists for all links    =  1.51663 [us]

> pose + jacobian + velocity twists for all links +  acceleration twists for all links               =  1.83826 [us]

>pose + jacobian + velocity twists for all links +  acceleration twists for all links               + jerk twists for all links                       =  2.68916 [us]

>pose + jacobian + velocity twists for all links +  acceleration twists for all links               + joint torque                                    =  3.76733 [us]

>pose + jacobian +  joint inertia matrix                                  = 10.06761 [us]


## Acknowledgements

RosDyn is developed by CNR-STIIMA (www.stiima.cnr.it)

***
<!--
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png"
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg"
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287.
