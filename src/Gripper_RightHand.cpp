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

// Action interface type for moving TIAGo's RIGHT gripper, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_control_client;
typedef boost::shared_ptr< gripper_control_client> gripper_control_client_Ptr;

//global definitions
ros::ServiceClient CloseGripperCLient;
std_srvs::Empty emptyObj;

// flag values
bool new_hand_position = false;
bool hand_closed;

// Create a ROS action client to move TIAGo's RIGHT gripper
void createRightGripperClient(gripper_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to gripper controller ...");

  actionClient.reset( new gripper_control_client("/parallel_gripper_right_controller/follow_joint_trajectory") );

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

// Generates a simple trajectory with one waypoint to open TIAGo's gripper 
void waypoints_openGripper_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names
  goal.trajectory.joint_names.push_back("gripper_right_left_finger_joint"); 
  goal.trajectory.joint_names.push_back("gripper_right_right_finger_joint"); 
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

void openRightGripper(gripper_control_client_Ptr& gripper_client)
{
  // Generates the goal for the TIAGo's gripper
  control_msgs::FollowJointTrajectoryGoal gripper_goal;
  waypoints_openGripper_goal(gripper_goal);

  // Sends the command to start the given trajectory 0s from now 
  gripper_goal.trajectory.header.stamp = ros::Time::now();
  gripper_client->sendGoal(gripper_goal);
  ROS_INFO("opening gripper...");  
}

// Callback to receive msgs from UNITY environment about hand gestures (open/closed)
void UnityCallback(const unity_robotics_demo_msgs::MyHandClosedConstPtr& msg )
{  
    hand_closed  = msg->hand_is_closed;
    new_hand_position = true;
}

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "right_hand_gripper_node");

  ROS_INFO("Starting MY right hand gripper node ...");
  
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0)))  // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

// Create a gripper controller action client to move the TIAGo's gripper
    gripper_control_client_Ptr GripperClient;
    createRightGripperClient(GripperClient);
  
   // Subscribe to unity topic to receive hand closed msgs
  ros::Subscriber gripper_sub = nh.subscribe("right_hand_grip_topic", 1, UnityCallback);
  
  //define the client for /parallel_gripper_right_controller/grasp
  CloseGripperCLient = nh.serviceClient<std_srvs::Empty>("/parallel_gripper_right_controller/grasp");
  
  // Main loop
  while (ros::ok())
  {
    // Check if a new joint state has been received
    if (new_hand_position)
    {
     ROS_INFO(" -> Hand Condition Changed ... <- ");
     
     if( hand_closed )
     {
     	ROS_INFO("###### Closing Hand  ########");     	
     	CloseGripperCLient.waitForExistence();
     	CloseGripperCLient.call(emptyObj);     	
     }

     else
     {
     	ROS_INFO("###### Opening Hand ########");
     	openRightGripper(GripperClient);  
     }
     
     // Reset the flag
     new_hand_position = false;      
    }

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  return EXIT_SUCCESS;
}

