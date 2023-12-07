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
* [Project structure](#project-structure)
* [Software Components and code description](#software-components-and-code-description)
* [Behaviuor Presentation](#behaviuor-presentation)
* [Limitations and Possible Improvements](#limitations-and-possible-improvements)


## Introduction



The Map
====================================================================

	
The Robot
====================================================================


	<!---
AAA
-->
<p><p align="center">
<img src="https://github.com/claudio-dg/assignment2/blob/main/media/My_robot1.png?raw=true" width="400" />
<p>
	
<p><p align="center">
<img src="https://github.com/claudio-dg/assignment2/blob/main/media/My_robot2.png?raw=true" width="400" />
<p>

	
Please find the code to create this robot in the [urdf](https://github.com/claudio-dg/assignment2/tree/main/urdf) folder.
##  Dependencies and Setup

In order to run correctly the project of this repository, some important dependencies have to be taken into account, therefore please make sure to have the following packages already installed in your ```ros_workspace```:
- First of all you will need to have the Pkg of my first assignment that you can find in this link: [assignment_1](https://github.com/claudio-dg/assignment_1/tree/second_assignment_changes). Please make sure to clone this repository and to be in the correct branch that i created that is ```second_assignment_changes```, where you can find the codes slightly modified in order to adapt them to this part of the project
- Additionally you will of course need the packages listed in the Readme of [assignment_1](https://github.com/claudio-dg/assignment_1/tree/second_assignment_changes), that for sake of simplicity I briefly list again here: [arch_skeleton](https://github.com/buoncubi/arch_skeleton), [topological_map](https://github.com/buoncubi/topological_map), [aRMOR](https://github.com/EmaroLab/armor) and [SMACH](http://wiki.ros.org/smach) libraries.
- **MoveBase** package, of the ROS Navigation stack :
```bash
$ sudo apt-get install ros-<ros_distro>-navigation
```
- [explore-lite](https://github.com/CarmineD8/m-explore) package
- [planning](https://github.com/CarmineD8/planning) package to use path planning algorithms
- [aruco_ros](https://github.com/CarmineD8/aruco_ros) package for being able to work with aruco markers.

	
When all these are correctly installed, to try the whole project it is necessary to: clone this repository in your ROS workspace: 

```bash
$ git clone https://github.com/claudio-dg/assignment2.git
```

then type the following commands in a terminal to make sure to launch first part of the project:

```bash
$ Roscore &
$ rosrun armor execute it.emarolab.armor.ARMORMainService

```
Please note that after building the package for the first time it may be required to go to the project directory and run the following command to correctly use aRMOR

```bash
$ ./gradlew deployApp
```
	
After that you can type the following command in another terminal:

```bash
$ roslaunch assignmnent_1 start_simulation.launch
```

In the end that, to launch the programms contained in this repository, simply open a new terminal and launch:
	
```bash
$ roslaunch assignmnent2 assignment.launch
```

These will open Gazebo and Rviz Simulation Environments along with a terminal showing the functioning of ```FSM```, and the robot will start its programmed behaviour
## Project structure

The Overall project is based on the ROS scheme that is shown in the following ```rqt_graph```:

<p align="center">
<img src="https://github.com/claudio-dg/assignment2/blob/main/media/final_rosgraph.png?raw=true" width="850" />
<p>
 
This repository contains a ROS package called ```"assignment2"``` that includes the following resources:

- [CMakeList.txt](https://github.com/claudio-dg/assignment2/blob/main/CMakeLists.txt): File to configure this package.
- [package.xml](https://github.com/claudio-dg/assignment2/blob/main/package.xml): File to configure this package.
- [Docs](https://github.com/claudio-dg/assignment2/tree/main/Docs): folder containing ```Doxygen documentation``` of the package
- [config](https://github.com/claudio-dg/assignment2/tree/main/config) : contains some parameters of robot's motors
- [media/](https://github.com/claudio-dg/assignment2/tree/main/media): folder containing images and graphs used within this [README](https://github.com/claudio-dg/assignment2/blob/main/README.md).
- [msg](https://github.com/claudio-dg/assignment2/tree/main/msg): contains a useful message for room information.
- [src](https://github.com/claudio-dg/assignment2/tree/main/src): It contains the implementation of each software components produced for this project.	
	* [marker_publish.cpp](https://github.com/claudio-dg/assignment2/blob/main/src/marker_publish.cpp): contains the implementation of Aruco markers detector and publisher
	
	* [marker_server.cpp](https://github.com/claudio-dg/assignment2/blob/main/src/marker_server.cpp): implements a service that requires the id (marker) detected by the robot and it replies with the information about the corresponding room (name of the room, coordinates of the center, connections with other rooms)

	
	* [move_arm_server.cpp](https://github.com/claudio-dg/assignment2/blob/main/src/move_arm_server.cpp): It contains the implementation of the server to move robot's arm

- [srv](https://github.com/claudio-dg/assignment2/tree/main/srv) : contains two useful services for roomInfo and robot pose.
- [urdf/](https://github.com/claudio-dg/assignment2/tree/main/urdf): folder containing robot Xacros to build the model and a urdf auto.generated but not used in this project
- [worlds/](https://github.com/claudio-dg/assignment2/tree/main/worlds): contains the simulation environment representing the "house" to be monitored

	
- [launch/](https://github.com/claudio-dg/assignment2/tree/main/launch): It contains the launch files to start the simulation
	
	* [gmapping.launch](https://github.com/claudio-dg/assignment2/blob/main/launch/gmapping.launch): launch file the planning algorithm
	
	* [move_base.launch](https://github.com/claudio-dg/assignment2/blob/main/launch/move_base.launch): launch file of the move_bsae package to move the robot
	
	* [assignment.launch](https://github.com/claudio-dg/assignment2/blob/main/launch/assignment.launch): launch file of the simulation that launches Gazebo and Rviz environments with the model of the robot plus the required nodes.

	

 
## Software Components and code description
	
Here are shown the details of each software component implemented in this repository and contained in the [src/](https://github.com/claudio-dg/assignment2/tree/main/src) folder.

```marker_publish``` node 
====================================================================

This node implements an algorithm that, exploiting ```Aruco``` Libraries, recognises the Id related to markers that are seen by the camera; to do so it subscribes to ```/image``` topic, but to apply it to my robot I remapped such topic to ```/robot/camera1/image_raw``` in the launch file (see [assignment.launch](https://github.com/claudio-dg/assignment2/blob/main/launch/assignment.launch)). After detecting an aruco, it starts printing the related ID, and thanks to the part of the code that I added, it publishes it (only once) on ```/my_ID_topic```. When the last ID is detected (i.e. id=14), this node is aborted. The subscriber to such topic is to be found in the ```FSM``` of assignment_1 package, where the received ID is managed to retrieve room informations and buil the topological map.
	
Added part to this code:
```bash
          int current_id =  markers_.at(i).id;
          
          if(current_id != prec) 
	//if statement to publish the id only once, when a new aruco is detected instead having multiple publishes for the same marker
          {
          ROS_INFO("NUOVO ID E': %i ...",current_id );
          prec = current_id;
          
          id.data = current_id;
          id_pub.publish(id);
          if(prec ==14) 
          // terminate this node when last  marker is detected
 	  {
 	  ROS_INFO("All markers detected, terminate the node");
 	  ros::Duration(2).sleep();
 	  abort();
 	  }
          }
```	
	
	
	
```marker_server``` node  
====================================================================
	
This node implements a server for ```/room_info``` that receives the number of ID detected from robot's camera, and  gives as answer the information of the room related (the name, the center coordinates etc...). In addition to this part, that was already given by the professor's code, I added the client to ```/MY_move_arm```, so that each time a marker is detected, such service is called to move the robot's arm towards the successive pre-defined aruco marker position.
	
	
	
	
	
	
```move_arm_server``` node  
====================================================================

This node implements the server of ```/MY_move_arm``` to move robot's arm : when called it will publish the joint values on the various ```Joint_position_controllers``` it is subscribed to, according to the request number, in order to move the arm to a certain pose, (each related to a certain aruco marker to detect). In particular it presents 9 different poses to select from, 1 for each marker plus a default pose and a pose to rotate the camera of 360 degrees.

```bash
bool reach(assignment2::MY_SetPose::Request &req, assignment2::MY_SetPose::Response &resp){

switch (req.pose_number){
	case 0:
		
		ROS_INFO("Default pose");
		joint1_value.data =0;
		joint2_value.data =0;
		joint3_value.data =0.1;
		joint4_value.data =0;
		new_pose = 1;
		break;
	case 1: 
		
		ROS_INFO("first marker pose");
		joint1_value.data =0.2;
		joint2_value.data =1.6;
		joint3_value.data =0;
		joint4_value.data =-1.5;
		new_pose = 1;
		break;
	case 2:
		
		ROS_INFO("second marker pose");
		joint1_value.data =0.2;
		joint2_value.data =0.8;
		joint3_value.data =0.5;
		joint4_value.data =-1.5;
		new_pose = 1;
		break;
	case 3: 
		
		ROS_INFO("third marker pose");
		joint1_value.data =2.15; //1.95
		joint2_value.data =0.8;
		joint3_value.data =0.5;
		joint4_value.data =-1.5;
		new_pose = 1;
		break;
	case 4:
		
		ROS_INFO("fourth marker pose");
		joint1_value.data =3.14;//3.14
		joint2_value.data =0.8;
		joint3_value.data =0.5;
		joint4_value.data =-1.5;
		new_pose = 1;
		break;
	case 5: 
		
		ROS_INFO("fifth marker pose");
		joint1_value.data =3.14;
		joint2_value.data =1.6;
		joint3_value.data =0;
		joint4_value.data =-1.5;
		new_pose = 1;
		break;
	case 6:
	
		ROS_INFO("sixth marker pose");
		joint1_value.data =3.8;
		joint2_value.data =1.6;
		joint3_value.data =0;
		joint4_value.data =-1.5;
		new_pose = 1;
		break;
	case 7: 
		
		ROS_INFO("seventh marker pose");
		joint1_value.data =4.6;//4.4
		joint2_value.data =1.3;
		joint3_value.data =0;
		joint4_value.data =0;
		new_pose = 1;
		break;
	case 10: 
		
		ROS_INFO("Rotation request");
		sign = -sign ;  //+-1
		joint1_value.data =6.28*sign; //3.6
		joint2_value.data =0;
		joint3_value.data =0.1;
		joint4_value.data =0;		
		new_pose = 1;
		break;
	default:
		ROS_INFO("Non existing Pose");
		
		break;
		}
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_arm_server");
  ros::NodeHandle nh;
  //init service server
  ROS_INFO("Creating Service server to move arm  ...");
  ros::ServiceServer service = nh.advertiseService("MY_move_arm", reach);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  // Define the publishers for robot's joints 
  pub_joint1 = nh.advertise<std_msgs::Float64> ("/m2wr/joint1_position_controller/command", 1);
  pub_joint2 = nh.advertise<std_msgs::Float64> ("/m2wr/joint2_position_controller/command", 1);
  pub_joint3 = nh.advertise<std_msgs::Float64> ("/m2wr/joint3_position_controller/command", 1);
  pub_joint4 = nh.advertise<std_msgs::Float64> ("/m2wr/joint4_position_controller/command", 1);
  
  //At the beginning force the position of first marker, then wait for calls 
  joint1_value.data =0.2;
  joint2_value.data =1.6;
  joint3_value.data =0;
  joint4_value.data =-1.5;
  ROS_INFO("START MOVING towards the first marker pose in 3 sec...");
  int i = 0;
  ros::Duration(3).sleep();
  pub_joint1.publish(joint1_value);
  pub_joint2.publish(joint2_value);
  pub_joint3.publish(joint3_value);
  pub_joint4.publish(joint4_value);

  while(ros::ok()){
  	 
 	 if (new_pose)
 	 {
		
		pub_joint1.publish(joint1_value);
		pub_joint2.publish(joint2_value);
		pub_joint3.publish(joint3_value);
		pub_joint4.publish(joint4_value); 
		ROS_INFO("PUBLISHING: J1= %f** J2 = %f** J3=%f** J4= %f...",joint1_value.data,joint2_value.data,joint3_value.data,joint4_value.data );
		new_pose = 0;
	 }
  }
  return(0);
}	
```
 

## Behaviuor Presentation
 
Markers Detection DEMO 
====================================================================
The following video (please click on the image to see it) shows the first part of the simulation in which the robot detects one by one the aurco markers: each time a marker is detected, the robot builds the ontology of the map adding the information retrived by the Aruco, and then moves towards another marker until the last one is detected; at that point it moves to a default pose and starts with the patrolling algorithm described in the next section.

[![Click to see Simulation Demo](https://github.com/claudio-dg/assignment2/blob/main/media/ArucoIntro.png?raw=true)](https://youtu.be/vXN9BJobEvc)


Patrolling DEMO 
====================================================================
	
	
Here you can find a short video (by clicking on the image) showing the resulting simulation of the second part of my project, that is the patrolling part that comes after the markers detection. Please notice that most of the video has a x10 speed up and also that some numbers will Pop-Up in the screen to highlight the crucial moments of the simulation: for a better understanding such numbers are asscoiated to a text description of the behaviour that you can find here just below the video.

[![Click to see Simulation Demo](https://github.com/claudio-dg/assignment2/blob/main/media/Intro.png?raw=true)](https://youtu.be/ZzqchcErcfk)

**1.** After having detetcted all markers, the robot plans to move to C2 and starts its motion, waiting for the goal to be reached or for the battery to get low.
	
**2.** The robot has reached the goal, since it is a corridor it does not apply survey algorithm, but plans to move to R3 Room and starts doing it.
	
**3.** Once R3 is reached the robot rotates the camera of 360 degrees, and plans to move back to the previous corridor.
	
**4.** Before reaching the exact coordinates of C2 the batery gets low: robot enters in Recharging state of the Finite state machine, and since he hasn't reached yet the corridor it believes to be still in R3; for this reason it plans again to reach C2, and once it reaches its exact coordinates it plans to reach the recharging station and moves towards it.
	
**5.** In the end the robot reaches the Recharging station and starts recharging the battery. It waits until it gets full charge, and then restarts its routine by moving towards a corridor.
	
	
	
 ## Limitations and Possible Improvements
 
The **main limitations** of this project are related to the speed of the robot while patrolling the map, and in general to some "imperfect" motions it makes which, for instance, forced me to increase the minimum range of detection of its laser scan in order to avoid detecting the ground as obstacle (due to chassis balancing) and therefore avoid building erroneous local costmap that caused the robot to reach its goal with a lot of difficulties.
	
Therefore **possible improvements** in this sense could involve the usage of a different model of robot more optimised with respect to the one I build, or try to apply some changes to the used one to improve its performances.
	
In addition to this, the implementation of the ```robot battery``` could be further improved, since for now it is just represented by a boolean value changing randomly its value after some predefined time, so a better algoritmh could be used to discharge the robot accordingly with the ```travelled distance``` for instance, which would also avoid the robot to reach locations that are too far from its recharging station, that would prevent him to go back and recharge its battery. Moreover another limition about it is that it could happen that robot may actually receive the "recharged" battery state before actually reaching the Recharging station, therefore an algorithm based on "travelled distance" could help in this case. 

In the end, one last limitation is related to the "room reaching activity": in fact since a room is considered reach only when its exact given coordinates are reached, if a robot gets low battery when already phisically in a room, but not yet to the given point, it believes to be still in the previous one. This happened also in the demo previously shown, but as said id didn't affect the functioning of the simulation, so it is not a big issue actually.
