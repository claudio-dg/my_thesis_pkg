my_thesis_pkg
================================
This repository contains the ROS Software components developed for my master thesis in Robotics engineering, to implement a telerobotic system to control TIAGo robot from remote.

#### Author: Claudio Del Gaizo, S4696649

#### mail: cdg9914@gmail.com


Table of contents
----------------------

* [Introduction](#introduction)
* [Dependencies and Setup](#dependencies-and-setup)
* [Content Description](#content-description)


## Introduction
This repository contains all the software component developed on ROS for my master thesis in Robotics engineering, to implement a telerobotic system to control TIAGo robot from remote.
The following sections are divided as follows:
   * **Dependencies and Setup**: offers some links to the required packages for this application, and explains how to deploy this application on your PC-
   * **Content Description**: explains the content of the folders contained in this repository.

## Dependencies and Setup

In order to make this repository work properly, make sure to have the here listed dependencies correctly installed in your ROS-Workspace.

* [TIAGo++ robot packages](http://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/Installation/Installing_Tiago%2B%2B_tutorial_docker): follow this tutorial to install required packages to interact with TIAGo, both in Gazebo simulation and with the real robot. For personal experience these installations may not always work, so manual installation may be required. 
  
* [ROS-TCP Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint): ROS package used to create an endpoint to accept ROS messages sent from a Unity scene. This ROS package works in tandem with the [ROS TCP Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector/tree/main) Unity package.

* [unity_robotics_demo_msg](https://github.com/claudio-dg/unity_robotics_demo_msgs.git): package containing the messages required both from the ROS side and the Unity counterpart, as they are used to let these two endpoints communicate with each other. Make sure to have them installed on your ROS ws and on your system running Unity Engine.

* [Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main): visit this repository to find useful tutorials on how to implement ros-unity interaction using formely listed packages.
 


Once all these dependencies are correctly included in your workspace, you can download this repository with the following command line:

```bash
$ git clone https://github.com/claudio-dg/my_thesis_pkg.git
```
Then you can launch the project using the [launchFile](https://github.com/claudio-dg/my_thesis_pkg/blob/main/launch/scripts.launch), which will run all the ROS nodes, including the ROS server endpoint to interact with Unity counterpart. 

```bash
$ roslaunch my_thesis_pkg scripts.launch

```
In order to launch correctly the server endpoint make sure to modify the [server.py](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/blob/main/src/ros_tcp_endpoint/server.py) script in the ROS-TCP Endpoint package, by modifying line 49 with the current IP address of your PC. To obtain your addres type in a terminal the following command.

```bash
$ hostname -I

```


## Content Description

Here follows the folders' description:

* [docs](https://github.com/claudio-dg/my_thesis_pkg/tree/main/docs): contains the Doxygen documentation of the code developed. Visit the following [link](https://claudio-dg.github.io/my_thesis_pkg/files.html) to have direct access to the documentation.
* [launch](https://github.com/claudio-dg/my_thesis_pkg/tree/main/launch): contains the launch file to start the project.
* [src](https://github.com/claudio-dg/my_thesis_pkg/tree/main/src): contains the C++ codes developed.
  * ```Gripper_LeftHand.cpp``` and ```Gripper_rightHand.cpp``` are TIAGo's grippers controllers;
  * ```clientLEFT_arm.cpp``` and ```clientRIGHT_arm.cpp``` are TIAGo's arms controllers;
  * ```move_head_joint.cpp``` is TIAGo's head controller;
  * ```my_ik_server.cpp``` is the server node to compute inverse kinematics;
* [srv](https://github.com/claudio-dg/my_thesis_pkg/tree/main/srv): contains custom service ```MyInverseKinematic.srv``` used to let arms controllers interact with the IK server.
  
