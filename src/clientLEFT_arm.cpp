// C++ standard headers
#include <exception>
#include <string>
// Boost headers
#include <boost/shared_ptr.hpp>
// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
// custom srv header
#include <my_thesis_pkg/MyInverseKinematic.h>
// unity msg headers
#include <unity_robotics_demo_msgs/MyPosRot.h>
#include <unity_robotics_demo_msgs/PosRot.h>
#include <tf/tf.h>

// Action interface type for moving TIAGo's ARM, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr<arm_control_client> arm_control_client_Ptr;

// Global definitions
ros::ServiceClient ikClient;
ros::ServiceClient limitless_ikClient;
my_thesis_pkg::MyInverseKinematic goal_frame;
bool new_frame_received = false;
float prev_roll = 0;
float prev_pitch = 0;
float prev_yaw = 0;

// Callback to receive msgs from UNITY environment about hand joint Pose
void UnityCallback(const unity_robotics_demo_msgs::PosRotConstPtr& msg )
{  
  // fill the client request with the values received
  goal_frame.request.arm = "left";
  goal_frame.request.end_effector_frame.position.x = msg->pos_x; 
  goal_frame.request.end_effector_frame.position.y = msg->pos_y; 
  goal_frame.request.end_effector_frame.position.z = msg->pos_z; 
  
  // Convert received quaternions into RPY
  tf::Quaternion q(msg->rot_x, msg->rot_y, msg->rot_z, msg->rot_w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
  // modify roll value in case of TIAGo's pal-5-Hand to have coherency with users' hand
  //roll = roll - 1.57;  
  
  goal_frame.request.end_effector_frame.orientation.x = roll;
  goal_frame.request.end_effector_frame.orientation.y = pitch; 
  goal_frame.request.end_effector_frame.orientation.z = yaw;
  
  ROS_INFO("rotation values: roll = %f pitch = %f yaw= %f ", roll, pitch, yaw);
  new_frame_received = true;
}


// Create a ROS action client to move TIAGo's arm
void createArmClient(arm_control_client_Ptr& action_client, const std::string& arm_controller_name)
{
  ROS_INFO("Creating action client for %s...", arm_controller_name.c_str());

  std::string action_client_name = "/" + arm_controller_name + "/follow_joint_trajectory";
  action_client.reset(new arm_control_client(action_client_name));

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while (!action_client->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations)
  {
    ROS_DEBUG("Waiting for the %s action server to come up", arm_controller_name.c_str());
    ++iterations;
  }

  if (iterations == max_iterations)
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}

// Move the arm to the desired joint state 
void moveArmToJointState(const control_msgs::FollowJointTrajectoryGoal& joint_goal, arm_control_client_Ptr& action_client)
{
  control_msgs::FollowJointTrajectoryGoal goal;
  goal = joint_goal;  
  goal.trajectory.header.stamp = ros::Time::now();   
  // Send the goal to the action server
  action_client->sendGoal(goal);
}


// function that takes desired frame's positions contained in the global variable 'goal_frame' to
// call the 'my_ik_solver_service' and returns as output the joint values contained in the service response
control_msgs::FollowJointTrajectoryGoal computeIK() 
{
control_msgs::FollowJointTrajectoryGoal resulting_joint_goal; 
// call 'my_ik_solver_service' to receive joint position required to reach 'goal_frame'
ikClient.waitForExistence();
ikClient.call(goal_frame);
ROS_INFO("calling ik_solver_service ...");

// in case of empty response, call again the same service passing as request the same position but with previous (successfull) rotation components
if(goal_frame.response.joint_positions.empty())
{
  ROS_INFO("\n * Receveived empty response from ik_server..not reachable pos ... * ");
  ROS_INFO("Calling AGAIN the service PASSING former Rotation values...\n");
  
  goal_frame.request.end_effector_frame.orientation.x = prev_roll;
  goal_frame.request.end_effector_frame.orientation.y = prev_pitch; 
  goal_frame.request.end_effector_frame.orientation.z = prev_yaw;
  ROS_INFO(" Calling AGAIN the service with  ROLL = %f pitch = %f yaw= %f ", prev_roll, prev_pitch, prev_yaw);
  
  ikClient.waitForExistence();
  ikClient.call(goal_frame);
  
  // if response is empty again do nothing
  if(goal_frame.response.joint_positions.empty())
  {  
  ROS_INFO("\n * Receveived AGAIN empty response from ik_server..not reachable pos ... * ");
  }  
}
else //else response is NOT empty 
{
//update previous rotation values
prev_roll = goal_frame.request.end_effector_frame.orientation.x;
prev_pitch = goal_frame.request.end_effector_frame.orientation.y;
prev_yaw = goal_frame.request.end_effector_frame.orientation.z;
ROS_INFO(" saved ROLL = %f pitch = %f yaw= %f ", prev_roll, prev_pitch, prev_yaw);
}

// if response is empty here, return predefined flag value
if(goal_frame.response.joint_positions.empty()) 
{	 
	 // resize to avoid segmentation fault
	 resulting_joint_goal.trajectory.points.resize(1);
	 resulting_joint_goal.trajectory.points[0].positions.resize(7);	 
   // fill variable with predefined flag value
	 resulting_joint_goal.trajectory.points[0].positions[0] = 200;
  
	 return resulting_joint_goal;
}
// else fill the variable with obtained joints values
else
{

// fill resulting_joint_goal fields
resulting_joint_goal.trajectory.header.stamp = ros::Time::now(); 
resulting_joint_goal.trajectory.joint_names.push_back("arm_left_1_joint");
resulting_joint_goal.trajectory.joint_names.push_back("arm_left_2_joint");
resulting_joint_goal.trajectory.joint_names.push_back("arm_left_3_joint");
resulting_joint_goal.trajectory.joint_names.push_back("arm_left_4_joint");
resulting_joint_goal.trajectory.joint_names.push_back("arm_left_5_joint");
resulting_joint_goal.trajectory.joint_names.push_back("arm_left_6_joint");
resulting_joint_goal.trajectory.joint_names.push_back("arm_left_7_joint");


resulting_joint_goal.trajectory.points.resize(1);

// First and ONLY trajectory point
  int index = 0;
  resulting_joint_goal.trajectory.points[index].positions.resize(7);
  resulting_joint_goal.trajectory.points[index].positions[0] = goal_frame.response.joint_positions[0];
  resulting_joint_goal.trajectory.points[index].positions[1] = goal_frame.response.joint_positions[1];
  resulting_joint_goal.trajectory.points[index].positions[2] = goal_frame.response.joint_positions[2];
  resulting_joint_goal.trajectory.points[index].positions[3] = goal_frame.response.joint_positions[3];
  resulting_joint_goal.trajectory.points[index].positions[4] = goal_frame.response.joint_positions[4];
  resulting_joint_goal.trajectory.points[index].positions[5] = goal_frame.response.joint_positions[5];
  resulting_joint_goal.trajectory.points[index].positions[6] = goal_frame.response.joint_positions[6];
   
  resulting_joint_goal.trajectory.points[index].velocities.resize(7); 
  for (int j = 0; j < 7; ++j)
     {
      // set low velocities for smooth and accurate movements
      resulting_joint_goal.trajectory.points[index].velocities[j] = 0.1; 
     }
  
  resulting_joint_goal.trajectory.points[index].time_from_start = ros::Duration(0.8);
  return resulting_joint_goal;
  }
}

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "client_ik_publisher_left");
  ROS_INFO("Starting left arm application ...");
  
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0)))  // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Create an arm left controller action client to move the TIAGo's left arm
  arm_control_client_Ptr arm_left_client;
  createArmClient(arm_left_client, "arm_left_controller");

  // initialze server client to /my_ik_solver_service
  ikClient = nh.serviceClient<my_thesis_pkg::MyInverseKinematic>("my_ik_solver_service");
  
  
  ros::Subscriber frame_sub = nh.subscribe("left_arm_frame_topic", 1, UnityCallback);
  //define variable
  control_msgs::FollowJointTrajectoryGoal resulting_joint_goal;
  
  // Main loop
  while (ros::ok())
  {
    // Check if a new joint state has been received from unity
    if (new_frame_received)
    {
     // copmute the inverse kinematic for the new frame calling compteIK() function
     resulting_joint_goal = computeIK();
     
     if( resulting_joint_goal.trajectory.points[0].positions[0] == 200 )
     {
     	ROS_INFO("###### result joint EMPTY!!! ########");
     }
     else
     {
     	// Move the arm to the new desired joint state
      moveArmToJointState(resulting_joint_goal, arm_left_client);
      ROS_INFO("###### moving right arm  #######"); 
     }    
     // Reset the flag
     new_frame_received = false;
    }
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  return EXIT_SUCCESS;
}
