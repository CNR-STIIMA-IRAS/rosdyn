![](Documentation/rosdyn_logo.png)


ROSdyn implements a fully automated procedure able to calibrate the robot dynamics model. 

It is integrated with MoveIt! to automatically compute, simulate, and execute identification trajectory. The result is stored in a URDF file.




## Build/Installation 

The software can be installed with the following [rosinstall file](rosdyn.rosinstall). 


## Build Status

| Kinetic | Melodic |
| --------|-------- |
| [![Build Status](https://travis-ci.org/CNR-STIIMA-IRAS/rosdyn.svg?branch=master)](https://travis-ci.org/CNR-STIIMA-IRAS/rosdyn) | [![Build Status](https://travis-ci.org/CNR-STIIMA-IRAS/rosdyn?branch=melodic-devel)](https://travis-ci.org/CNR-STIIMA-IRAS/rosdyn) |


## List of packages

> *rosdyn_core*: Dynamics header library based on Eigen. With respect to KDL, it has two advantages: it is faster and it allows to compute model regressor.

> *rosdyn_identification*: Nodelet-based library for trajectory generation and for model calibration.

> *rosdyn_gui*: RViz front-end plugin

> *rosdyn_identification_msgs*: Action definition for generating identification trajectory and estimating model.

## Usage

__ coming soon! __

![](Documentation/screenshoot001.png)


## Work in progress

ROSdyn is continuously evolving. Not all features are implemented. If find errors or you wish a new feature [please let us know](https://github.com/CNR-STIIMA-IRAS/rosdyn/issues).

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

