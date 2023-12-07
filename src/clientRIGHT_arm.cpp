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

//includo il custom srv *
//#include <tiago_dual_moveit_tutorial/MyInverseKinematic.h>
#include <my_thesis_pkg/MyInverseKinematic.h>

//provo a includere il msg per Unity -->modifico CmakeList inserendo unity_robotics_demo_msgs in find package a inzio
#include <unity_robotics_demo_msgs/MyPosRot.h>
//#include <unity_robotics_demo_msgs/PosRot.h>

// Our Action interface type for moving TIAGo's ARM, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr<arm_control_client> arm_control_client_Ptr;


/* ############################################################################################

qui per ira 1° step  = voglio solo implementare un client che mandi una posizione FRAME predefinita al server
(quindi non ricevuta ancora ma settata manualmente), prende risposta e la usa per muovere braccio robot

poi 2° step = fare in modo che pos mandata dal client sia quella ricevuta tramite sub da un nodo di UNITY

CONSULTARE PROF PER CAPIRE SE HA SENSO DIVIDERE QUESTIONEDEI DUE NODI SUB diversi per LE DUE BRACCIA
quindi se conviene pubblicare su un topic solo (distinguendo left/right) e poi avere un solo nodo sub che mandi
richiesta singola a server
OPPURE se conviene pubblicare su due nodi diversi e quindi avere due sub in due nodi separati...

############################################################################################*/

// Global variables

//creo client globale
ros::ServiceClient ikClient;
ros::ServiceClient limitless_ikClient;
//creo oggetto per la request
//tiago_dual_moveit_tutorial::MyInverseKinematic goal_frame;
my_thesis_pkg::MyInverseKinematic goal_frame;


//per vedere quanto tempo passa
ros::Time previous_msg_time;

//////// control_msgs::FollowJointTrajectoryGoal resulting_joint_goal; ############


bool new_frame_received = false;

// Callback function for the joint states ----> DEVE DIVENTARE CALLBACK PER IL FRAME ( +nome braccio oppure mi servono 2 nodi per questione che avevo detto del farli muovere contemporanemante???) PUBBLICATO DA UNITY
/*
void jointStateCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& msg)
{
  desired_joint_state = *msg;
  new_joint_state_received = true;
}
*/

//questa sarà la calbback del sub al topic 'braccio dx'su cui unity pubblicherà pos desiderata frame (da creare custom_msg x questo)
/* OLD ONE FUNZIONANTE
void futureCallback(const geometry_msgs::PoseConstPtr& msg ) //(const control_msgs::FollowJointTrajectoryGoalConstPtr& msg)
{
  // desired_frame = *msg; qui dovrei inizializzare il frame desiderato con quello ricevuto da unity, per ora manualmente lo setto
  
  //sta parte sotto quindi va poi cancellata
  //riempo la richiesta del client con il valore che ricevo qua (setto ora a mano)
  goal_frame.request.arm = "right";
  goal_frame.request.end_effector_frame.position.x = msg->position.x; //0.217; //qui poi dovro mettere tipo msg->posX na roba cois, riempo coi campi del messaggio
  goal_frame.request.end_effector_frame.position.y = msg->position.y; //-0.228;
  goal_frame.request.end_effector_frame.position.z = msg->position.z; //-0.476;//0.35;
  goal_frame.request.end_effector_frame.orientation.x = msg->orientation.x; //orient poi da gestire, per ora il server le resetta come vuole lui
  goal_frame.request.end_effector_frame.orientation.y = msg->orientation.y;
  goal_frame.request.end_effector_frame.orientation.z = msg->orientation.z;
  goal_frame.request.end_effector_frame.orientation.w = msg->orientation.w;
  
  ROS_INFO("callbackFrame--ricevuto nuovo frame con x =  %f...", msg->position.x);
  new_frame_received = true;
}
*/

//provo a implementare vera callback
void UnityCallback(const unity_robotics_demo_msgs::MyPosRotConstPtr& msg )
{
  /* 
  PosRot:
float32 pos_x
float32 pos_y
float32 pos_z
float32 rot_x
float32 rot_y
float32 rot_z
float32 rot_w
  
  */
  
  ros::Time current_time = ros::Time::now();
  double time_elapsed = (current_time - previous_msg_time).toSec();
  // Stampa il tempo trascorso tra i due messaggi in secondi
  ROS_INFO("\n ENTRO CALLBACK --> Tempo trascorso tra due messaggi: %.3f secondi\n", time_elapsed);
  // Aggiorna il tempo del messaggio precedente con il tempo corrente
  previous_msg_time = current_time;
  
  
  //riempo la richiesta del client con il valore che ricevo qua
  goal_frame.request.arm = "right";
  goal_frame.request.end_effector_frame.position.x = msg->pos_x; //0.217; //qui poi dovro mettere tipo msg->posX na roba cois, riempo coi campi del messaggio
  goal_frame.request.end_effector_frame.position.y = msg->pos_y; //-0.228;
  goal_frame.request.end_effector_frame.position.z = msg->pos_z; //-0.476;//0.35;
  goal_frame.request.end_effector_frame.orientation.x = msg->roll; //orient poi da gestire---> provo lasciando geom_msg/Pose anche se sarebbero quaternioni ma amen, io so che dentro a x metto roll, y=pitch ecc...poi non importa...lato server per prendere il roll estraggo quel campo li ecc...
  goal_frame.request.end_effector_frame.orientation.y = msg->pitch;
  goal_frame.request.end_effector_frame.orientation.z = msg->yaw;
  //goal_frame.request.end_effector_frame.orientation.w = msg->rot_w;
  
  ROS_INFO("ricevuto nuovo frame con x =  %f...", msg->pos_x);
  

  new_frame_received = true;
}
// Create a ROS action client to move TIAGo's arm --> questo pezzo rimane uguale, dovro poi chiamare 2 volte se faccio un nodo solo per entrambe le braccia, altrimentirimane cosi e faccio due nodi separati,,uno per braccio dx e uno per braccio sx
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

// Move the arm to the desired joint state --> qui dentro devo inserire la chiamata al servizio IK e prendere la risposta contenente pos desiderata giunti, OPPURE!! LASCIO QUESTA INVARIATA E FACCIO UN'ALTRA funzione che in cui c'è
//chiamata a servizio ik...e semplicemente la chiamo nel main prima di questa funzione moveArmToJointState...
void moveArmToJointState(const control_msgs::FollowJointTrajectoryGoal& joint_goal, arm_control_client_Ptr& action_client)
{
  control_msgs::FollowJointTrajectoryGoal goal;
  goal = joint_goal;

  
  goal.trajectory.header.stamp = ros::Time::now(); //qui serve rimettere questo /serve solo qua forse?

 
 
 // PROVA DI STAMPA NOMI GIUNTI GOAL --> sono corretti...
 
 for (int j = 0; j < 7; ++j)
 {
    //ROS_INFO("nomi joint goal: n %i = %s*",j,goal.trajectory.joint_names[j].c_str());
    ROS_INFO("pos joint goal: n %i = %f \n",j,goal.trajectory.points[0].positions[j]);
  }
  
  // Send the goal to the action server
  action_client->sendGoal(goal);
}


// ----- FUNZIONE PER IK ------

control_msgs::FollowJointTrajectoryGoal computeIK() //lo passo come argomento goal_frame o lo lascio cosi globale? forse come argomento è piu leggibile il codice
{
 control_msgs::FollowJointTrajectoryGoal resulting_joint_goal;
//la risposta del servizio la metto poi dentro a var "resulting_joint_goal" ? 
ikClient.waitForExistence();
ikClient.call(goal_frame);
ROS_INFO("calling ik_solver_service ...");

// ******** CHIAMATA ALTERNATIVA AD ALTRO SERVIZIO SENZA Joint_limits SE QUESTO FALLISCE ********
if(goal_frame.response.joint_positions.empty())
{
  ROS_INFO("\n ** Receveived empty response from ik_server..not reachable pos ... ** ");
  ROS_INFO("Calling alternative service to receive acceptable values...\n");
  limitless_ikClient.waitForExistence();
  limitless_ikClient.call(goal_frame);
  
}
/////// ** /////else{ 

//forse in questo modo dato che dovrei ricevere in ogni caso ina risposta non serve piu l'else..
//riempo oggetto resulting_joint_goal con risposta del client
//resulting_joint_goal // per riempirlo pero devo mettere qui anche i joint names manualmente ecc vedi prova pub ( i nomi potrei inizializzarli nel main
//goal_frame.response
//costruisco ogg FollowJointTrajGoal usando priori knowledge e response del server

resulting_joint_goal.trajectory.header.stamp = ros::Time::now(); //questa NON puo andare nel main
resulting_joint_goal.trajectory.joint_names.push_back("arm_right_1_joint");//ste cose volendo le metto nel main cosi fa una volta sola?
resulting_joint_goal.trajectory.joint_names.push_back("arm_right_2_joint");
resulting_joint_goal.trajectory.joint_names.push_back("arm_right_3_joint");
resulting_joint_goal.trajectory.joint_names.push_back("arm_right_4_joint");
resulting_joint_goal.trajectory.joint_names.push_back("arm_right_5_joint");
resulting_joint_goal.trajectory.joint_names.push_back("arm_right_6_joint");
resulting_joint_goal.trajectory.joint_names.push_back("arm_right_7_joint");


resulting_joint_goal.trajectory.points.resize(1);

// First and ONLY trajectory point
  // Positions
  int index = 0;
  resulting_joint_goal.trajectory.points[index].positions.resize(7);
  resulting_joint_goal.trajectory.points[index].positions[0] = goal_frame.response.joint_positions[0];//1.0; //0.0
  resulting_joint_goal.trajectory.points[index].positions[1] = goal_frame.response.joint_positions[1];
  resulting_joint_goal.trajectory.points[index].positions[2] = goal_frame.response.joint_positions[2];//0.4;
  resulting_joint_goal.trajectory.points[index].positions[3] = goal_frame.response.joint_positions[3];
  resulting_joint_goal.trajectory.points[index].positions[4] = goal_frame.response.joint_positions[4];
  resulting_joint_goal.trajectory.points[index].positions[5] = goal_frame.response.joint_positions[5];
  resulting_joint_goal.trajectory.points[index].positions[6] = goal_frame.response.joint_positions[6];
   
  resulting_joint_goal.trajectory.points[index].velocities.resize(7); //ste cose volendo le metto nel main cosi fa una volta sola?
  for (int j = 0; j < 7; ++j)
     {
    resulting_joint_goal.trajectory.points[index].velocities[j] = 1.0; //---> POSSO SETTARE VELOCITA DIVERSE PER I VARI GIUNTI : ESEMPIO IL PRIMO GIUNTO CHE MAGARI FA MOVIMENTI PIU AMPI PIU VELOCE MENTRE QUELLI FINALI PIU LENTI PER MOVIMENTO PIU SMOOTH??
     }
  
  resulting_joint_goal.trajectory.points[index].time_from_start = ros::Duration(1.25);
  return resulting_joint_goal;
  /////// ** ///// }
}
// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "client_ik_e_publisher_posizione_right");

  ROS_INFO("Starting MY_run_dual_traj_control application ...");
  
  

  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0)))  // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  // Create an arm right controller action client to move the TIAGo's right arm
  arm_control_client_Ptr arm_right_client;
  createArmClient(arm_right_client, "arm_right_controller");

  // Subscribe to the desired joint state topic
  //ros::Subscriber joint_state_sub = nh.subscribe("my_topic", 1, jointStateCallback);

   // Inizializzione del client per il servizio "my_ik_solver_service"
   //ikClient = nh.serviceClient<tiago_dual_moveit_tutorial::MyInverseKinematic>("my_ik_solver_service");
   
   ikClient = nh.serviceClient<my_thesis_pkg::MyInverseKinematic>("my_ik_solver_service");
   
    // Inizializzione del client per il servizio "limitless_ik_service" da chiamare in caso quello con joint limits restituisca pos non reachable
   limitless_ikClient = nh.serviceClient<my_thesis_pkg::MyInverseKinematic>("limitless_ik_service");

// limitless_ikClient = nh.serviceClient<tiago_dual_moveit_tutorial::MyInverseKinematic>("limitless_ik_service");



   // Subscribe to the desired joint state topic
  // ros::Subscriber frame_sub = nh.subscribe("right_arm_frame_topic", 1, futureCallback);
  ros::Subscriber frame_sub = nh.subscribe("right_arm_frame_topic", 1, UnityCallback);
  
  control_msgs::FollowJointTrajectoryGoal resulting_joint_goal;
  
  // Inizializza il tempo del messaggio precedente con il tempo corrente
  previous_msg_time = ros::Time::now();
  
  //chiamo future callback qui per dare inizio al procedimento...
  //in implementazione vera poi avro  callback vera che da inizio da sola
 ///////////////// futureCallback(); //**cancellare poi sta riga
  // Main loop
  while (ros::ok())
  {
    // Check if a new joint state has been received
    if (new_frame_received)
    {
    
     ROS_INFO("CURRENT request.X = %f",goal_frame.request.end_effector_frame.position.x);
     resulting_joint_goal = computeIK();
      // Move the arm to the new desired joint state
      moveArmToJointState(resulting_joint_goal, arm_right_client);
      ROS_INFO("###### input ricrvuto ... MUOVO BRACCIO DESTRO  ... ########");
      // Reset the flag
      new_frame_received = false;
    }

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return EXIT_SUCCESS;
}
