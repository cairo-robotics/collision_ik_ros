# README Template

## Description and Purpose

Maximum 1 paragraph about the repo and/or project. Include:
- Whom it is for (public/lab/personal)
- What the current state/stage of the project is (completed/abandoned/in-development)
- Any published works related to this repo

---

## Table of Contents

- [Dependencies](#dependencies)
- [System Overview](#system-overview)
- [Important Features](#important-ros-topics-apis-functions)
- [Installation](#installation)
- [Running the Project](#running-the-project)
- [FAQs and Issues](#faq-and-common-issues)
- [Contributions, Copyright, License, Citations](#contributions-copyright-license-citations)
- [Acknowledgments](#acknowledgments)

---
## Dependencies

### ROS Dependencies
ROS Noetic

### Python Dependencies
**readchar**:
```bash
sudo pip install readchar
```
**fcl collision library**: https://github.com/BerkeleyAutomation/python-fcl
```bash
sudo pip install python-fcl
```
**scipy**:
```bash
sudo pip install scipy
```
**Pyyaml**:
```bash
sudo pip install PyYaml
```
**Pypcd**: https://github.com/dimatura/pypcd
Lastly, update your version of **numpy**:
```bash
sudo pip install --upgrade numpy
```

### Rust Dependencies
To use this wrapper, you will first need to install Rust. Please go to https://www.rust-lang.org/learn/get-started for more infomation.


---
## System Overview

This repository acts as a ROS1 wrapper around the cairo_planning_core repository, which is CAIRO labs extension to the RelaxedIK/CollisionIK software developed by Daniel Rakita and the UW graphics group. 

---
## Important Features

1. topic/function and message content/function I/O
2. etc. 
3. etc.

Provide links to specific files in your repo as appropriate, so that users know where these things are within your code.

---
## Installation

1. Make sure that you have installed all the dependencies.
1. Clone this repo to the *src* directory in your catkin workspace.
1. Build your catkin workspace by using `catkin_make` or `catkin build` (if you have catkin-tools installed) in the workspace root directory. 
1. Clone the plannig_core_rust from the CAIRO lab organization. It doesn't matter where you clone this as you will be installing this as a Python package locally. 
1. Navigate to the the cloned repository folder and go through the steps below to get ensure the package ready for use.
    1. If your robot is in this list: [baxter, hubo, iiwa7, jaco7, panda, sawyer, ur5, yumi], ignore this step. Else, you will need to clone [this repo](https://github.com/uwgraphics/relaxed_ik) and follow the step-by-step guide [there](https://github.com/uwgraphics/relaxed_ik/blob/dev/src/start_here.py) to get the required robot config files into corresponding folders in the *config* folder in the core. To specify, there should be (replace "sawyer" with your robot name or your urdf name in some cases):
        - 1 self-collision file <collision_sawyer.yaml> in the *collision_files* folder
        - 4 Rust neural network files <sawyer_nn, sawyer_nn.yaml, sawyer_nn_jointpoint, sawyer_nn_jointpoint.yaml> in the *collision_nn_rust* folder
        - 1 info file <sawyer_info.yaml> in the *info_files* folder
        - 1 joint state function file <sawyer_joint_state_define> in the *joint_state_define_functions* folder
        - 1 urdf file <sawyer.urdf> in the *urdfs* folder.
   
1. Look at <settings.yaml> in the folder *cairo_planning_core/data/config* and follow the instructions there to customize the parameters, change the input device, and manage the environment obstacles. Note that you don't need to recompile/reinstall the package every time you change the parameters in <settings.yaml>.

1. Install the planning_core_rust package either as a system python package or (ideally) into a virtual environment. A good strategy is to create your virtual environment and the update the `activate` script with the following:
    
    ```export PYTHONPATH='PATH_TO_YOUR_CATKIN_WORKSPACE/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages::/usr/lib/python3/dist-packages:'PATH_TO_YOUR_CATKIN_WORKSPACE/venv/lib/python3.6/site-packages'
    ```

Then execute:

    ```
    $ pip3 install PATH_TO_PLANNING_CORE_RUST_REPOSITORY
    ```
---
## Running the Project

1. View the robot arm in rviz by typing the following command:
    ```bash
    roslaunch relaxed_ik_ros1 rviz_viewer.launch
    ```
1. Launch the Relaxed IK solver by typing the following command:
    ```bash
    roslaunch relaxed_ik_ros1 relaxed_ik_rust.launch
    ```
1. Set a ROS parameter to start the simulation:
    ```bash
    rosparam set /simulation_time go
    ```
1. **[For Testing purposes]** Control the robot based on the type of input device in <settings.yaml>. If you set the robot follow a given cartesian trajectories, you should see the robot moving now! Some examples of cartesian trajectories are provided in the folder *animation_files*. If you set the `input_device` to be keyboard, initialize the keyboard IK goal driver in a new terminal. The driver listens to 6-DOF pose goals and publishes robot joint angles.
    ```bash
    rosrun relaxed_ik_ros1 keyboard_ikgoal_driver.py
    ```
    To use the keyboard controller, please ensure that the termainal window where <keyboard_ikgoal_driver.py> was run from has focus (i.e., make sure it's clicked), then use the following keystrokes:
    ```bash
    c - kill the controller controller script
    w - move chain 1 along +X
    x - move chain 1 along -X
    a - move chain 1 along +Y
    d - move chain 1 along -Y
    q - move chain 1 along +Z
    z - move chain 1 along -Z
    1 - rotate chain 1 around +X
    2 - rotate chain 1 around -X
    3 - rotate chain 1 around +Y
    4 - rotate chain 1 around -Y
    5 - rotate chain 1 around +Z
    6 rotate chain 1 around -Z

    i - move chain 2 along +X
    m - move chain 2 along -X
    j - move chain 2 along +Y
    l - move chain 2 along -Y
    u - move chain 2 along +Z
    n - move chain 2 along -Z
    = - rotate chain 2 around +X
    - - rotate chain 2 around -X
    0 - rotate chain 2 around +Y
    9 - rotate chain 2 around -Y
    8 - rotate chain 2 around +Z
    7 - rotate chain 2 around -Z
    ```

### Config changes

n/a

### Examples of running the project

n/a
---
## FAQs and Issues

- This wrapper only support the XYZ-like point cloud file format currently. Please refer to the geometry files in the folder *geometry_files* for some examples.

---
## Contributions, Copyright, License, Citations

If you want to make contributions to this project, what to do.

Copyright and license info.

If you use this project, please cite it as ___.

---
## Acknowledgments

https://pages.graphics.cs.wisc.edu/relaxed_ik_core/
https://github.com/uwgraphics/relaxed_ik_core

