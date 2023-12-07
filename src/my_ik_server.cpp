// ROS headers
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
// Inverse Kinematics headers
#include <kdl/chain.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp> // to create ChainIkSolverPos_NR_JL
#include <kdl/chainiksolvervel_pinv.hpp>  // to create KDL::ChainIkSolverVel_pinv
#include <kdl/chainfksolverpos_recursive.hpp>  // to create KDL::ChainFkSolverPos_recursive
//Custom srv header
#include <my_thesis_pkg/MyInverseKinematic.h>

std::string base_link = "torso_lift_link";

    std::string left_end_link = "arm_left_tool_link"; 
    std::string right_end_link = "arm_right_tool_link"; 

    // Create 2 KDL Chains using base link and end link
    KDL::Chain left_kdl_chain;
    KDL::Chain right_kdl_chain;

    // Initialise joint limits and positions as KDL Joint Arrays
    int number_of_joints = 7; 

    KDL::JntArray left_joint_limits_min(number_of_joints);
    KDL::JntArray left_joint_limits_max(number_of_joints);

    KDL::JntArray right_joint_limits_min(number_of_joints);
    KDL::JntArray right_joint_limits_max(number_of_joints);
    
    KDL::JntArray left_joint_positions_initial(number_of_joints);
    KDL::JntArray right_joint_positions_initial(number_of_joints);

    KDL::JntArray left_joint_positions_result(number_of_joints);
    KDL::JntArray right_joint_positions_result(number_of_joints);


   //Frames to be set containing the desired Frame Position 
   KDL::Frame right_end_effector_desired;
   KDL::Frame left_end_effector_desired; 

// Callback to update joint states info
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
 // Access message's fields
  std::vector<std::string> joint_names = msg->name; //only used for debugging
  std::vector<double> joint_positions = msg->position;

  // Update current arm positions
  for (int j = 0; j < number_of_joints; ++j) 
  {
   left_joint_positions_initial(j) = msg->position[j]; //left range from  0 to 6
   right_joint_positions_initial(j) = msg->position[j+7]; //right range from 7 to 13
  }

}

//callback to my service
bool IKSolver(my_thesis_pkg::MyInverseKinematic::Request& req, my_thesis_pkg::MyInverseKinematic::Response& res)
{
   // Create left ik and fk solvers required by KDL::ChainIkSolverPos_NR_JL
   KDL::ChainIkSolverVel_pinv left_ik_solver_vel(left_kdl_chain);  
   KDL::ChainFkSolverPos_recursive left_fk_solver(left_kdl_chain);  
   
   // Create right ik and fk solvers required by KDL::ChainIkSolverPos_NR_JL
   KDL::ChainIkSolverVel_pinv right_ik_solver_vel(right_kdl_chain);  
   KDL::ChainFkSolverPos_recursive right_fk_solver(right_kdl_chain);

   // Create left and right KDL::ChainIkSolverPos_NR_JL
   KDL::ChainIkSolverPos_NR_JL left_ik_solver(left_kdl_chain, left_joint_limits_min, left_joint_limits_max, left_fk_solver,left_ik_solver_vel );
   KDL::ChainIkSolverPos_NR_JL right_ik_solver(right_kdl_chain, right_joint_limits_min, right_joint_limits_max, right_fk_solver,right_ik_solver_vel );

    // take RPY data received by the client
    double roll,pitch,yaw;
    roll = req.end_effector_frame.orientation.x;
    pitch = req.end_effector_frame.orientation.y;
    yaw = req.end_effector_frame.orientation.z;

    ROS_INFO("RPY RECEIVED BY CLIENT : roll = %f pitch = %f yaw = %f  ", roll,pitch,yaw);

    //if left arm pose requested
    if(req.arm == "left") 
    {

     left_end_effector_desired.p.x(req.end_effector_frame.position.x);
     left_end_effector_desired.p.y(req.end_effector_frame.position.y);
     left_end_effector_desired.p.z(req.end_effector_frame.position.z);
 
    left_end_effector_desired.M =  KDL::Rotation::RPY(roll,pitch,yaw); 
     
     //compute inverse kinematics for the left arm
     int result = left_ik_solver.CartToJnt(left_joint_positions_initial, left_end_effector_desired, left_joint_positions_result);

    // if inverse kinematics failed return error
    if (result < 0) {
        ROS_ERROR("Failed to calculate joint positions");
        const char* errorMessage = left_ik_solver.strError(result);
        std::cout << "########## Error: ########" << errorMessage << std::endl;

        return 1;
    }

    // otherwise inverse kinematics computed successfully => fill the response and print the results
    ROS_INFO("*********** FINAL Joint Positions: *************");
    for (int i = 0; i < left_joint_positions_result.rows(); ++i) 
    {

            res.joint_positions.push_back(left_joint_positions_result(i));
            ROS_INFO("LEFT Joint number %i --> POSITION %f",i ,res.joint_positions[i]);
    }
    } 

    //else if right arm pose requested
    else if (req.arm == "right")
    {
     right_end_effector_desired.p.x(req.end_effector_frame.position.x);
     right_end_effector_desired.p.y(req.end_effector_frame.position.y);
     right_end_effector_desired.p.z(req.end_effector_frame.position.z);
     
    right_end_effector_desired.M =  KDL::Rotation::RPY(roll,pitch,yaw);
    
     //compute inverse kinematics for the right arm
     int result = right_ik_solver.CartToJnt(right_joint_positions_initial, right_end_effector_desired, right_joint_positions_result);
         // if inverse kinematics failed return error
         if (result < 0) {
            ROS_ERROR("Failed to calculate joint positions");
            const char* errorMessage = right_ik_solver.strError(result);
                 std::cout << "########## Errore: ########" << errorMessage << std::endl;
            return 1;
        }

        // otherwise inverse kinematics computed successfully => fill the response and print the results
        ROS_INFO("*********** FINAL Joint Positions: *************");
    for (int i = 0; i < right_joint_positions_result.rows(); ++i) 
    {
            res.joint_positions.push_back(right_joint_positions_result(i));
            ROS_INFO("RIGHT Joint number %i --> POSITION %f",i ,res.joint_positions[i]);
    }
    }
    else
    {
     ROS_ERROR("Wrong service Request received, please specify the arm typing <left> or <right> as argument of your client call... ");
    }

    return true;
}

 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ik_solver_server");
    ros::NodeHandle nh;

    //create server for my custom service of ik 
    ros::ServiceServer service = nh.advertiseService("my_ik_solver_service", IKSolver);

    // Subscribe to  joint_states topic obatin joint states in real time
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStateCallback);

    ROS_INFO("Server del servizio IK Solver pronto per la chiamata.");

      KDL::Tree kdl_tree;

    // load URDF model of the robot
    urdf::Model robot_model;
   if (!robot_model.initParam("robot_description")) {
        ROS_ERROR("Failed to parse URDF model");
        return 1;
    }


    // Convert URDF model into KDL Tree
    if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree)) {
        ROS_ERROR("Failed to extract KDL tree from URDF model");
        return 1;
    }

    // Get the chains of links from the trees between base_link and end_link
    if (!kdl_tree.getChain(base_link, left_end_link, left_kdl_chain)) {
        ROS_ERROR("Failed to extract LEFT KDL chain from KDL tree");
        return 1;
    }

    if (!kdl_tree.getChain(base_link, right_end_link, right_kdl_chain)) {
        ROS_ERROR("Failed to extract RIGHT KDL chain from KDL tree");
        return 1;
    }

   // Define MIN and MAX joint limits for LEFT arm
    left_joint_limits_min(0) = -1.10; //arm 1 
    left_joint_limits_min(1) = -1.11;
    left_joint_limits_min(2) = -0.78;
    left_joint_limits_min(3) = -0.39;
    left_joint_limits_min(4) = -2.09;
    left_joint_limits_min(5) = -1.39;
    left_joint_limits_min(6) = -2.09;

    left_joint_limits_max(0) = 1.50;
    left_joint_limits_max(1) = 1.50;
    left_joint_limits_max(2) = 3.92;
    left_joint_limits_max(3) = 2.35;
    left_joint_limits_max(4) = 2.09;
    left_joint_limits_max(5) = 1.39;
    left_joint_limits_max(6) = 2.09;

    // Define MIN and MAX joint limits for RIGHT arm
    right_joint_limits_min(0) = -1.10; //arm 1 
    right_joint_limits_min(1) = -1.11;
    right_joint_limits_min(2) = -0.78;
    right_joint_limits_min(3) = -0.39;
    right_joint_limits_min(4) = -2.09;
    right_joint_limits_min(5) = -1.39;
    right_joint_limits_min(6) = -2.09;
   
    right_joint_limits_max(0) = 1.50;
    right_joint_limits_max(1) = 1.50;
    right_joint_limits_max(2) = 3.92;
    right_joint_limits_max(3) = 2.35;
    right_joint_limits_max(4) = 2.09;
    right_joint_limits_max(5) = 1.39;
    right_joint_limits_max(6) = 2.09; 

    ros::spin();
    return 0;
}

