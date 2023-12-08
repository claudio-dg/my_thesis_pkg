my_thesis_pkg
================================
This repository contains the ROS Software components developed for my master thesis in Robotics engineering, to implement a telerobotic system to control TIAGo robot from remote.

my repo containing msgs for unity ros communication, make sure to have this package both on ros side and unity counterpart [unity_robotics_demo_msg](https://github.com/claudio-dg/unity_robotics_demo_msgs.git)
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

* [unity_robotics_demo_msg](https://github.com/claudio-dg/unity_robotics_demo_msgs.git): This package contains the messages required both from the ROS side and the Unity counterpart, as they are used to let these two endpoints communicate with each other. Make sure to have them installed on your ROS ws and on your system running Unity Engine.
 
* [TIAGo++ robot packages](http://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/Installation/Installing_Tiago%2B%2B_tutorial_docker): follow this tutorial to install required packages to interact with TIAGo, both in Gazebo simulation and with the real robot. For personal experience these installations may not always work, so manual installation may be required. 

## Content Description

