/*
*******************************************************************************
*                            DATA ACQUISITION SYSTEMS, S.A.
*                             PANAMA, REPUBLIC OF PANAMA
*
*  File          : main.c
*  Progremmer(s) : Rangel Alvarado
*  Language      : C++
*  Description   :
*  ----------------------------------------------------------------------------
*  HISTORY
*  MM DD YY
*  07 26 20 Created.
*  07 30 20 Added comments.
*******************************************************************************
*/

/*
*******************************************************************************
*                     NECCESARY HEADER FILES FOR THE APPLICATION
*******************************************************************************
*/
#include <ros/ros.h>               // include ros essentials package
#include <geometry_msgs/Twist.h>   // for turtlesim msg of topic cmd_vel 
#include <geometry_msgs/Pose.h>    // for turtlesim msg of topic pose
#include <turtlesim/Pose.h>        // for turtlesim topic cmd_vel
#include <turtlesim/Spawn.h>       // for turtlesim service spawn
#include <turtlesim/Kill.h>        // for turtlesim service kill
#include <math.h>                  // for math function atan2

/*
*******************************************************************************
*                                   NAMESPACES
*******************************************************************************
*/


using namespace std;                        // std namespace for cout i.e.

/*
*******************************************************************************
*                                 GLOBAL VARIABLES
*******************************************************************************
*/

//Create the pose variable
turtlesim::Pose turtle_pose;                // to store the pose of the turtle

//Create the publisher and subscribers
ros::Publisher cmd_vel_pub;                 // to publish comand of velocity
ros::Subscriber pose_sub;                   // to subscribe to the pose

//Create the client services
ros::ServiceClient spawn_client;            // to add a spawn service
ros::ServiceClient kill_client;             // to add a kill service

//Create the request and response objects of spawn.
turtlesim::Spawn::Request spawn_req;        // need a req. and resp. for spawn
turtlesim::Spawn::Response spawn_resp; 

//Create the request and response objects of kill.
turtlesim::Kill::Request kill_req;          // need a req. and resp. for kill
turtlesim::Kill::Response kill_resp;


/*
*******************************************************************************
*                            FUNCTION PROTOTYPES
*******************************************************************************
*/

// callback of the pose of the chasing turtle
void poseCallback(const turtlesim::Pose::ConstPtr &pose_msg);

// function to move the turtle to a goal pose due to a tolerance of finalziation
void moveTo(turtlesim::Pose goal_pose, double tolerancem);

// A PID model for distnce
double PID_distance(std::vector<double> Ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt);

// A PID model for angle
double PID_angle(std::vector<double> Ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt);

// To spawn a 'leader' turtle
void spawnTurtle(turtlesim::Pose leader_pose);

// To kill the current spawned turtle 'leader'
void killTurtle(void);


/*
*******************************************************************************
*                                 MAIN PROGRAM
*******************************************************************************
*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "moveTurtle");  // Initialize the move turtle with ROS
  ros::NodeHandle n;                    // Create the handler of the turtle1 node

  // Publish the cmd_vel with a buffer size of 10
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

  // Subscribe to the pose and deliver the states to the 'poseCallback' function
  pose_sub = n.subscribe("/turtle1/pose", 10, poseCallback);

  // make the service to the spawn client
  spawn_client = n.serviceClient<turtlesim::Spawn>("/spawn");

  // make the service to the kill client
  kill_client = n.serviceClient<turtlesim::Kill>("/kill");

  turtlesim::Pose goal_pose;          // local variable to the goal pose  
  while(true) {                       // chasing turtles forever
    // make the random pose to be between 0-12
    goal_pose.x = random() % 12;      
    goal_pose.y = random() % 12;
    
     
    spawnTurtle(goal_pose);           // spawn a new turtle and make that the 
                                      // goal position
    moveTo(goal_pose, 0.1);       // do microsteps to move the turtle
    killTurtle();                     // kill the leader turtle
  }
  return 0;                           // exit the main function
}


/*
*******************************************************************************
*                              SUPPORT FUNCTIONS
*******************************************************************************
*/

/*
*******************************************************************************
*
*               MOVE A TURTLE TO A GOAL POSITION AND FINISHES
*                    WHEN A MINIMAL TOLERANCE IS REACHED
*
*  Description : Move the turtle to a current best position
*  Func. Name  : moveTo
*  Arguments   : turtlesim::Pose pose
*                    holds the current pose of the turtle
*                double tolerance
*                    defines when to stop the movement
*  Returns     : None
*  Notes       : None
*******************************************************************************
*/

void moveTo(turtlesim::Pose finish_pose, double tolerance) {
    geometry_msgs::Twist cmd_vel_msg;
    ros::Rate loop_rate(10);
    vector<double> KpKiKd_distance{1.01, 0.2, 0.001};
    vector<double> KpKiKd_angle{3.5, 0.05, 0.05};        
    double d;
    double dt;
    ros::Time start = ros::Time::now();
    ros::Time end = ros::Time::now() + ros::Duration(0.1);
    do {
        end = ros::Time::now();
        dt = end.toSec() - start.toSec();
        dt = dt < (1.0/10)*(0.05)?0.1:dt; // if the dt is 95% below 0.1 clip to 0.1
        cout << dt << endl;
        d = PID_distance(KpKiKd_distance, finish_pose, turtle_pose, 1.0/10);
        start = ros::Time::now();
        cmd_vel_msg.linear.x = d; 
        cmd_vel_msg.angular.z = PID_angle(KpKiKd_angle, finish_pose, turtle_pose, 1.0/10);
        cmd_vel_pub.publish(cmd_vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    } while (d > tolerance);
    cmd_vel_msg.linear.x = 0;
    cmd_vel_msg.angular.z = 0;
    cmd_vel_pub.publish(cmd_vel_msg);      
}


/*
*******************************************************************************
*
*               THE CALLBACK FUNCTION TO STORE THE POSE BY THE
*                             POSE SUBSCRIBER REQUEST
*
*  Description : Callback that holds the current pose of the turtle
*  Func. Name  : moveTo
*  Arguments   : const turtlesim::Pose::ConstPtr &pose_msg
*                    it has the current pose published by the turtlesim node
*  Returns     : None
*  Notes       : None
*******************************************************************************
*/

void poseCallback(const turtlesim::Pose::ConstPtr &pose_msg) {
    // same as...: turtle_pose.theta = pose_msg->theta; /*and so on for other vars*/
    turtle_pose = *pose_msg; 
}


/*
*******************************************************************************
*
*                  CALCULATE THE PID CONTROL SIGNAL FOR DISTANCE
*                                    CORRECTION
*
*  Description : Given the values of Kp, Ki, Kd, error and time delta to the 
*                controller calculates the control signal to apply
*  Func. Name  : PID_distance
*  Arguments   : std::vector<double> Ks
*                    a vector in the order Kp, Ki and Kd
*                turtlesim::Pose setpoint_pose
*                    the the final pose that we want to reach
*                turtlesim::Pose turtle_pose
*                    the current pose of the robot
*                double dt
*                    the time difference in seconds
*  Returns     : double 
*                    the control signal to apply to the plant
*  Notes       : None
*******************************************************************************
*/

double PID_distance(std::vector<double> Ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt) {
    turtlesim::Pose e;
    static double prev_error;
    static double derror;
    
    double Kd = Ks.back(); Ks.pop_back();
    double Ki = Ks.back(); Ks.pop_back();
    double Kp = Ks.back(); Ks.pop_back();

    e.x = setpoint_pose.x - turtle_pose.x;
    e.y = setpoint_pose.y - turtle_pose.y;
    double error = sqrt(pow(e.x,2) + pow(e.y,2));

    derror = error - prev_error;    
    double u = Kp*error + Ki*error*dt + Kd*derror/dt;
    prev_error = error;
    return u;
}


/*
*******************************************************************************
*
*                  DOUBLE THE PID CONTROL SIGNAL FOR ANGLE
*                                    CORRECTION
*
*  Description : Given the values of Kp, Ki, Kd, error and time delta to the 
*                controller calculates the control signal to apply
*  Func. Name  : PID_angle
*  Arguments   : std::vector<double> Ks
*                    a vector in the order Kp, Ki and Kd
*                turtlesim::Pose setpoint_pose
*                    the the final pose that we want to reach
*                turtlesim::Pose turtle_pose
*                    the current pose of the robot
*                double dt
*                    the time difference in seconds
*  Returns     : double 
*                    the control signal to apply to the plant
*  Notes       : None
*******************************************************************************
*/

double PID_angle(std::vector<double> Ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt) {
    turtlesim::Pose e;
    static double prev_error;
    static double derror;

    double Kd = Ks.back(); Ks.pop_back();
    double Ki = Ks.back(); Ks.pop_back();
    double Kp = Ks.back(); Ks.pop_back();
    
    double error = atan2(setpoint_pose.y - turtle_pose.y, setpoint_pose.x - turtle_pose.x) - turtle_pose.theta;
    derror = error - prev_error;
    double u = Kp*error + Ki*error*dt + Kd*derror/dt;
    
    prev_error = error;
    
    u = u >  4*M_PI? 4*M_PI:u;
    u = u < -4*M_PI?-4*M_PI:u;
    return u;
}


/*
*******************************************************************************
*
*          SPAWN A TURTLE AT GIVEN A POSE AS AN ARGUMENT
*
*
*  Description : Spawn a turtle in the position given
*  Func. Name  : spawnTurtle
*  Arguments   : turtlesim::Pose leader_pose
*                    the pose to spawn a turtle named 'leader' at
*                    a given pose (x,y,yaw) 
*  Returns     : None
*  Notes       : None
*******************************************************************************
*/

void spawnTurtle(turtlesim::Pose leader_pose) {
   spawn_req.x = leader_pose.x;
   spawn_req.y = leader_pose.y;
   spawn_req.theta = leader_pose.theta;
   spawn_req.name = "Leader";
   ros::service::waitForService("/spawn", ros::Duration(5));
   bool success = spawn_client.call(spawn_req, spawn_resp);
   if(success){
       ROS_INFO_STREAM("Reborn like fenix turtle named " << spawn_resp.name);
   }else{
       ROS_ERROR_STREAM("Failed to spawn turtle named " << spawn_resp.name);
   }
}

/*
*******************************************************************************
*
*          KILL A TURTLE AT GIVEN A POSE AS AN ARGUMENT
*
*
*  Description : Kill a turtle named 'leader'
*  Func. Name  : killTurtle
*  Arguments   : None 
*  Returns     : None
*  Notes       : None
*******************************************************************************
*/

void killTurtle(void) {
   kill_req.name = spawn_resp.name;
   ros::service::waitForService("/kill", ros::Duration(5));
   bool success = kill_client.call(kill_req, kill_resp);
   if(success){
       ROS_INFO_STREAM("Killed beloved turtle named " << spawn_resp.name);
   }else{
       ROS_ERROR_STREAM("Failed to kill turtle named " << spawn_resp.name);
   } 
}
