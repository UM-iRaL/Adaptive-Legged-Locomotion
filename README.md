# Adaptive Legged Locomotion via Online Learning for Model Predictive Control

## Overview

This repo contains the simulaton code with Gazebo associated to our paper [Adaptive Legged Locomotion via Online Learning for Model Predictive Control](https://arxiv.org/abs/2510.15626) (accepted to IEEE Robotics and Automation Letters). The algorithm is built upon our previous paper [Simultaneous System Identification and Model Predictive Control with No Dynamic Regret](https://arxiv.org/abs/2407.04143). In the paper, we propose the algorithm SSI-MPC for adaptive legged locomotion, with simultaneous model learning of unknown dyanmics/disturbance in a self-supervised manner using only the data collected on-the-go (i.e., without offline training), and model predictive control. 

![image](https://github.com/UM-iRaL/Adaptive-Legged-Locomotion/blob/main/doc/front.png)


### License

The source code is released under a [MIT License](LICENSE).


### Citing

If you use this code in an academic context, please cite the following publications:

```
@article{zhou2025adaptive,
  title={Adaptive Legged Locomotion via Online Learning for Model Predictive Control},
  author={Zhou, Hongyu and Zhang, Xiaoyu and Tzoumas, Vasileios},
  journal={arXiv preprint arXiv:2510.15626},
  year={2025}
}


@article{zhou2025simultaneous,
  title={Simultaneous system identification and model predictive control with no dynamic regret},
  author={Zhou, Hongyu and Tzoumas, Vasileios},
  journal={IEEE Transactions on Robotics},
  year={2025},
  publisher={IEEE}
}
```

The simulator is based on [Quad-SDK](https://github.com/robomechanics/quad-sdk):
```
@inproceedings{norby2022quad,
  title={Quad-SDK: Full stack software framework for agile quadrupedal locomotion},
  author={Norby, Joseph and Yang, Yanhao and Tajbakhsh, Ardalan and Ren, Jiming and Yim, Justin K and Stutt, Alexandra and Yu, Qishun and Flowers, Nikolai and Johnson, Aaron M},
  booktitle={ICRA Workshop on Legged Robots},
  pages={1--5},
  year={2022},
  organization={sn}
}
```


## Installation

Please refer to the [Quad-SDK Wiki](https://github.com/robomechanics/quad-sdk/wiki/1.-Getting-Started-with-Quad-SDK) for installation, dependency, and unit testing information. This code requires ROS Noetic on Ubuntu 20.04. All other dependencies are installed with the included setup script.


## Usage
Note: all MPC algorithms are tested with GO2 robot.

Launch the simulation with:

```
roslaunch quad_utils quad_gazebo.launch
```

Stand the robot with:
```
rostopic pub /robot_1/control/mode std_msgs/UInt8 "data: 1"
```
Run the stack with twist input:
```
roslaunch quad_utils quad_plan.launch reference:=twist logging:=true
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot_1/cmd_vel
```
Run the stack with global planner:
```
roslaunch quad_utils quad_plan.launch reference:=gbpl logging:=true
```
Refer to the [Wiki](https://github.com/robomechanics/quad-sdk/wiki/2.-Using-the-Software) for more information on alternate usage.

## Notes

In `/nmpc_controller/src/quad_nlp_utils.cpp`
- load basic leg controller functions for the `Robot_NAME` platform

MATLAB
- If you use different number of random features in SSI-MPC, you will need additional dynamics files with name `<robot_name>_rf`. The MATLAB code for generating dynamics can be found under `nmpc_controller/scripts/`. Currently, the provided generated code has used 50 random features.

Adaptive MPC
- `Nominal MPC`, `L1 MPC`, and `SSI-MPC` can be selected in `local_planner.yaml`:
   ```
   control_type: # nominal, rf, l1
       type: nominal
       # type: rf
       # type: l1
   ``` 

External forces
- Constant forces can be applied in `local_planner.yaml`:
   ```
   experiment_params:
       add_force: false
       fx: 0.0  # kg
       fy: 0.0  # kg
       fz: 0.0  # kg
   ``` 
