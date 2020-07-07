# Robot Dynamics Model Calibration Toolbox developed by CNR-ITIA (www.itia.cnr.it)

The repository contains the implementation of the toolbox for the automatic dynamics model calibration, 
developed by the Institute of Industrial Technologies and Automation, of the National Research Council of Italy (CNR-ITIA).


# Functionalities and code organization

The package **itia_dynamics_parameters_identification** is organized into three software units. 

1) **METO trajectories generations** is the unit devoted to the generation of the exciting trajectories. 
The software takes into account the robot self collisions and the robot collisions with the environment generating collision free trajectories.

2) **METO trajectories execution** is the unit devoted to the execution of the exciting trajectories. 
At the same time all the data necessary for the parameters estimations is recorded and stored into a binary file.

3) **METO dynamics parameters estimation** is the unit devoted to the dynamics parameters estimation. The binary file related to an


## METO trajectories generations

The **MetoGenInterfaceNodelet** is the nodelet interface to the class **MetoTrjOptGenTrj** and to the class **MetoTrjOptCheckCollision**.

- The class **MetoTrjOptGenTrj** implement all the functionalities to generate the exciting trajectories. 

- The class **MetoTrjOptCheckCollision** implement all the functionalities necessary to verify with MoveIt if the generated trajectory is a collision-free trajectory. 



## METO trajectories execution

The **MetoExecInterfaceNodelet** is the nodelet interface to the class **MetoTrjExec**. 

- The class **MetoTrjExec** implement all the functionalities to execute an exciting trajectory on a robot, both real and virtual robot are allowed. During the trajectories execution the relevant data are recorded in a binary file.




## METO dynamics parameters estimation

The **MetoParEstimInterfaceNodelet** is the nodelet interface to the class **MetoParEstim**. 

- The class **MetoParEstim** implement all the functionalities to estimate the dynamics parameters set. The experimental data are extracted from the binary file. The dynamics parameters are automatically saved in a new URDF file. 




# Package Test and Usage

## To use the METO functionalities

1) luanch the **demo.launch** of the desired robot

2) launch the **test_meto.launch** to load all the functionalities of the METO library

3) depending on which unit is necessary

> 3.a) launch **testo_meto_gen.launch** to generate the excting trajectories (the generated trajectories will be saved into the Parameter Server )

> > * the configuration file for the exciting trajectories generation is **meto_gen_config.yaml**

> 3.b) launch **test_meto_exec.launch** to execute the trajectory on the robot. Pay attention to the parameter **stage_number** into the launch file, it determines which trajectory will be executed: use "stage1" or "stage2"

> > * the configuration file for the exciting trajectories execution is **meto_exec_config.yaml**. The parameter **log_topic_name** determines the topic that will be logged into the binary file.

> 3.c) launch **test_meto_par_estim.launch** to estimate the dynamics parameters and save the new URDF file


## To change the robot

In case of a new robot it is necessary to change the following launch files:

1) in the launcher **test_meto.launch** change the 2 file names where are reported the additionals info

2) in the launcher **testo_meto_gen.launch** change the **xml_name** according with the robot URDF file names

3) in the launcher **testo_meto_par_estim.launch** change the **xml_name** and the file name where are reported the additionals info

4) in the configuration file **meto_general_config.yaml** pay attention to the **controller_joints_name** that need to be coherent with the joint names of the URDF file


## To test the package 

The following nodes allow to test all the functionalities provided from the **itia_dynamics_parameters_identification** library:

- test_meto_gen_interface_node to test the exciting trajectories generation 

- test_meto_exec_interface_node to test the exciting trajectories execution

- test_meto_par_estim_interface_node to the dynamics model parameters estimation


## To add obstacles in the meto_load_scene

To add new obstacles or change the floor in the scene a rough method is to use the **meto_load_obstacles.cpp** where are reported some examples of obstacles.


# TO DO:

## meto_gen:

1) meto_gen: aggiungere generazione traiettorie per identificazione TOOL


## meto_exec:

1) meto_sim: aggiungere possibilità riproduzione traiettorie in un intervallo ben definito di tempo


## meto_par_estim:

1) meto_par_estim: risolvere problema con tool_link ipotesi getChild dell'ultimo joint che nel file additional_info

2) meto_par_estim: algoritmo di ottimizzazione per passaggio da set parametri ridotti al set completo con vincolo fisicità parametri


## meto_load_scene (not necessary)

1) load obstacles from XACRO

2) automatic generation of URDF from XACRO in a launcher file




## Developer Contact

**Authors:**   
- Enrico Villagrossi (enrico.villagrossi@stiima.cnr.it)  
- Manuel Beschi (<mailto:manuel.beschi@stiima.cnr.it>)  
 
_Software License Agreement (BSD License)_    
_Copyright (c) 2017, National Research Council of Italy, Institute of Industrial Technologies and Automation_    
_All rights reserved._