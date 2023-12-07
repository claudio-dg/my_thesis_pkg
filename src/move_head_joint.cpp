// C++ standard headers
#include <exception>
#include <string>
// Boost headers
#include <boost/shared_ptr.hpp>
// ROS headers
#include <geometry_msgs/Pose.h>
// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>
// unity msg header
#include <unity_robotics_demo_msgs/MyPosRot.h>

// Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_control_client;
typedef boost::shared_ptr< head_control_client>  head_control_client_Ptr;

// global variables
float roll, pitch, yaw;
float old_pitch = 0;
float old_yaw = 0;
bool different_rotation = false;
bool different_inclination = false;
bool new_frame_received = false;

// define head joints limits
float joint_min_limits[2] = {-1.24, -0.98}; //yaw=joint1 ; Pitch=joint2
float joint_max_limits[2] = {1.24, 0.72};


// Create a ROS action client to move TIAGo's arm
void createHeadClient(head_control_client_Ptr& action_client, const std::string head_controller_name)
{
  ROS_INFO("Creating action client to %s ...", head_controller_name.c_str());

  std::string action_client_name = "/head_controller/follow_joint_trajectory";
  action_client.reset( new head_control_client(action_client_name) );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !action_client->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the head_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createHeadClient: head controller action server not available");
}

// Callback to receive msgs from UNITY environment about VR Head Mounted Display rotation
// Receive RPY data from UNITY, then use Pitch & Yaw to control directly head joints
void UnityCallback(const unity_robotics_demo_msgs::MyPosRotConstPtr& msg )
{
  different_rotation = false;
  different_inclination = false;

  roll = msg->roll; // not used for TIAGo
  pitch = msg->pitch;// UP - DOWN
  yaw = msg->yaw;// RIGHT - LEFT
  
   // Manual check on joint values: set to min/max if limits exceeded
  if(pitch < joint_min_limits[1])
  {
   pitch = joint_min_limits[1];
   ROS_INFO("PITCH under LIM_MIN resetted to min\n");
  }
  else if(pitch > joint_max_limits[1])
  {
   pitch = joint_max_limits[1];
   ROS_INFO("PITCH above LIM_MAX resetted to max\n");
  }
  
  // Manual check on joint values: set to min/max if limits exceeded
  if(yaw < joint_min_limits[0])
  {
   yaw = joint_min_limits[0];
   ROS_INFO("YAW OLTRE LIM_MIN resettato a min\n");
  }
  else if(yaw > joint_max_limits[0])
  {
   yaw = joint_max_limits[0];
   ROS_INFO("YAW OLTRE LIM_MAX resettato a max\n");
  }
 

    ROS_INFO(" ****** RECEIVED : roll = %f pitch = %f yaw= %f  ******* ", roll,pitch,yaw);
  
    // check if pitch is different w.r.t previous one
    if( pitch < (old_pitch-0.0500) or pitch > (old_pitch+0.0500))
    {
     old_pitch = pitch;
     different_inclination = true;
    }
    // check if pitch is different w.r.t previous one
    if( yaw < (old_yaw-0.0500) or yaw > (old_yaw+0.0500))
    {     
     old_yaw = yaw;
     different_rotation = true;
    }
    // only update boolean if a new rotation or inclincation is detected (avoid publishing same/similar values in the 'main')
    if(different_inclination or different_rotation)
    {
      new_frame_received = true; 
    }    
}

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "head_joint_controller");

  ROS_INFO("Starting head_joint_controller application ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  //Subscribe to head_frame_topic to receive data from UNITY's publisher
  ros::Subscriber unity_frame_sub = nh.subscribe("head_frame_topic", 1, UnityCallback);

  // Create an head controller action client to move the TIAGo's Head
  head_control_client_Ptr head_client;
  createHeadClient(head_client, "head_controller");

  control_msgs::FollowJointTrajectoryGoal head_goal;

  // Send the command to start the given trajectory 
  head_goal.trajectory.header.stamp = ros::Time::now();  
  head_goal.trajectory.joint_names.push_back("head_1_joint");//Yaw
  head_goal.trajectory.joint_names.push_back("head_2_joint");//Pitch  
  head_goal.trajectory.points.resize(1);
  
  // First and ONLY trajectory point
  int index = 0;
  head_goal.trajectory.points[index].positions.resize(2);

  head_goal.trajectory.points[index].velocities.resize(2); 
  for (int j = 0; j < 2; ++j)
     {
    head_goal.trajectory.points[index].velocities[j] = 1.0; 
     }
     
    // To be reached 4 seconds after starting along the trajectory
    head_goal.trajectory.points[index].time_from_start = ros::Duration(0.4);

  
  while (ros::ok())
  {
    // only publish new positions
    if(new_frame_received)
    {
     // ROS_INFO("Moving head  POS : PITCH = %f ... YAW= %f",pitch,yaw);

     // Generate the goal for the TIAGo's head using Pitch & Yaw received
    head_goal.trajectory.points[index].positions[0] = yaw;
    head_goal.trajectory.points[index].positions[1] = pitch;
    head_goal.trajectory.header.stamp = ros::Time::now();    
    head_client->sendGoal(head_goal);
   
    // Reset flag
    new_frame_received = false;    
    }
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  } 
  return EXIT_SUCCESS;
}

