/**
* \file Gripper_LeftHand.cpp
* \brief TIAGo's left hand's gripper controller
* \author Claudio Del Gaizo
* \version 0.1
* \date 7/12/2023
* 
*
* \details
*
* Subscribes to: <BR>
*
* /left_hand_grip : to have info about user's left hand's gestures from Unity
*
*
* Action Client to: <BR>
*
* /parallel_gripper_left_controller/follow_joint_trajectory : to move TIAGo's left gripper
*
*
* Service client to: <BR>
*
* /parallel_gripper_left_controller/grasp : to perform grasping action with TIAGo's gripper
*
*
* Description:
*
* This simple node subscribes to the /left_hand_grip to receive data from Unity's publisher about the gestures of the user's left hand. Then uses the received information to close TIAGo's gripper calling /parallel_gripper_left_controller/grasp service, or to open it calling /parallel_gripper_left_controller/follow_joint_trajectory action service.
**/

// C++ standard headers
#include <exception>
#include <string>
// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>
#include <std_srvs/Empty.h>
// unity msg header
#include <unity_robotics_demo_msgs/MyHandClosed.h>

// Action interface type for moving TIAGo's LEFT gripper, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_control_client;
typedef boost::shared_ptr< gripper_control_client> gripper_control_client_Ptr;

//global definitions
ros::ServiceClient CloseGripperCLient;  ///< global client definition
std_srvs::Empty emptyObj; ///< global empty message definition

// flag values
bool new_hand_position = false; ///< global boolean to identify a different gesture received
bool hand_closed; ///< global boolean to identify hand closed/open


/**
* \brief function to create action client 
*
* \param action_client : gripper_control_client_Ptr& : the pointer to the action client
*
*
* \return void
*
* This function is called to creat a client to the /parallel_gripper_left_controller/follow_joint_trajectory action service, used to control TIAGo's left gripper
*
**/
/* ***************************************************************************/
void createLeftGripperClient(gripper_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to gripper controller ...");

  actionClient.reset( new gripper_control_client("/parallel_gripper_left_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for gripper controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the gripper_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createGripperClient: gripper controller action server not available");
}


/**
* \brief function to generate a trajectory for the gripper 
*
* \param goal : control_msgs::FollowJointTrajectoryGoal: the values of joints angles to control TIAGo's gripper
*
*
* \return void
*
* This function generates a simple trajectory with one waypoint to open TIAGo's gripper updating global variable 'goal'
*
**/
/* ***************************************************************************/ 
void waypoints_openGripper_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names
  goal.trajectory.joint_names.push_back("gripper_left_left_finger_joint"); 
  goal.trajectory.joint_names.push_back("gripper_left_right_finger_joint"); 
  goal.trajectory.points.resize(1);

  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(2);
  goal.trajectory.points[index].positions[0] = 0.09;
  goal.trajectory.points[index].positions[1] = 0.09;

  // To be reached 0.8 seconds after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(0.8);
  ROS_INFO("opening gripper");
 
}


/**
* \brief function to open TIAGo's gripper
*
* \param gripper_client : gripper_control_client_Ptr: the pointer to the action client
*
*
* \return void
*
* This function opens TIAGo's gripper, firstly by calling 'waypoints_openGripper_goal' function to generate the trajectory, the sending the result to the action client '/parallel_gripper_left_controller/follow_joint_trajectory'
*
**/
/* ***************************************************************************/
void openLeftGripper(gripper_control_client_Ptr& gripper_client)
{
  // Generates the goal for the TIAGo's gripper
  control_msgs::FollowJointTrajectoryGoal gripper_goal;
  waypoints_openGripper_goal(gripper_goal);

  // Sends the command to start the given trajectory 0s from now 
  gripper_goal.trajectory.header.stamp = ros::Time::now();
  gripper_client->sendGoal(gripper_goal);
  ROS_INFO("opening gripper...");  
}


// 
/**
* \brief Subscriber Callback for /left_hand_grip
* 
* \param msg : unity_robotics_demo_msgs::MyHandClosedConstPtr& : a bolean value stating user's hand state ( 0 = open/ 1 = closed)
*
*
* \return void
*
* 
* Callback function to receive msgs from UNITY environment about hand gestures (open/closed), and update global variables to inform the main function that a new hand gesture has been received, specifying if the user's hand is now open or closed.
*
**/
void UnityCallback(const unity_robotics_demo_msgs::MyHandClosedConstPtr& msg )
{   
  hand_closed  = msg->hand_is_closed;
  new_hand_position = true;
}

/**
* \brief main function
*
*
* Entry point. Initializes the node, the subscriber to /left_hand_grip_topic, the action client to open TIAGO's gripper and the service client to /parallel_gripper_left_controller/grasp to close it. Then in the main loop, it waits for a new gesture to be received, then sends a request to the correspondant service/action to open or close TIAGo's gripper.
*
**/
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "left_hand_gripper_node");

  ROS_INFO("Starting MY left hand gripper node ...");

  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0)))  // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

// Create a gripper controller action client to move the TIAGo's gripper
    gripper_control_client_Ptr GripperClient;
    createLeftGripperClient(GripperClient);
  
   // Subscribe to unity topic to receive hand closed msgs
  ros::Subscriber gripper_sub = nh.subscribe("left_hand_grip_topic", 1, UnityCallback);
  
  //define the client for /parallel_gripper_left_controller/grasp
  CloseGripperCLient = nh.serviceClient<std_srvs::Empty>("/parallel_gripper_left_controller/grasp");
  
  // Main loop
  while (ros::ok())
  {
    // Check if a new joint state has been received
    if (new_hand_position)
    {
     ROS_INFO(" -> Hand Condition Changed ... <- ");
     
     if( hand_closed )
     {
     	ROS_INFO("###### Closing Hand ########");
     	CloseGripperCLient.waitForExistence();
     	CloseGripperCLient.call(emptyObj);     	
     }
     
     else
     {
     	ROS_INFO("###### Opening Hand ########");
     	openLeftGripper(GripperClient);     
     }
     
     // Reset the flag
     new_hand_position = false;      
    }

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  return EXIT_SUCCESS;
}

