#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include "math.h"

using namespace std;

turtlesim::Pose turtle_pose;

ros::Publisher cmd_vel_pub;
ros::Subscriber pose_sub;

ros::ServiceClient spawn_client;
ros::ServiceClient kill_client;

//Create the request and response objects.
turtlesim::Spawn::Request spawn_req;
turtlesim::Spawn::Response spawn_resp;
turtlesim::Kill::Request kill_req;
turtlesim::Kill::Response kill_resp;


void poseCallback(const turtlesim::Pose::ConstPtr &pose_msg);
void moveTo(turtlesim::Pose goal_pose, double tolerance);
double PID_distance(std::vector<double> Ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt);
double PID_angle(std::vector<double> Ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt);
void spawnTurtle(turtlesim::Pose leader_pose);
void killTurtle(void);


int main(int argc, char **argv) {
  ros::init(argc, argv, "moveTurtle");
  ros::NodeHandle n;
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
  pose_sub = n.subscribe("/turtle1/pose", 10, poseCallback);
  spawn_client = n.serviceClient<turtlesim::Spawn>("/spawn");
  kill_client = n.serviceClient<turtlesim::Kill>("/kill");

  
  turtlesim::Pose goal_pose;
  ros::Rate loop_rate(0.2);
  while(true) {
    goal_pose.x = random() % 12;
    goal_pose.y = random() % 12;
    spawnTurtle(goal_pose);
    moveTo(goal_pose, 0.1);
    killTurtle();

  }
  return 0;
}


void moveTo(turtlesim::Pose finish_pose, double tolerance) {
    geometry_msgs::Twist cmd_vel_msg;
    ros::Rate loop_rate(10);
    vector<double> KpKiKd_distance{0.12, 0.083, 0.01};
    vector<double> KpKiKd_angle{2.8, 0.05, 0.004};        
    double d;

    do {
        d = PID_distance(KpKiKd_distance, finish_pose, turtle_pose, 1.0/10);
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

void poseCallback(const turtlesim::Pose::ConstPtr &pose_msg) {
    turtle_pose = *pose_msg; // same as...: turtle_pose.theta = pose_msg->theta; /*and so on for other vars*/
}


double PID_distance(std::vector<double> Ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt) {
    static double prev_error_d = 0;
    turtlesim::Pose error;
    double Kd = Ks.back(); Ks.pop_back();
    double Ki = Ks.back(); Ks.pop_back();
    double Kp = Ks.back(); Ks.pop_back();

    error.x = setpoint_pose.x - turtle_pose.x;
    error.y = setpoint_pose.y - turtle_pose.y;
    double error_d = sqrt(pow(error.x,2) + pow(error.y,2));

    double u = Kp*error_d + Ki*error_d*dt + Kd*(error_d - prev_error_d)/dt;
    prev_error_d = error_d;
    return u;
}

double PID_angle(std::vector<double> Ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt) {
    static double prev_error_theta;
    double Kd = Ks.back(); Ks.pop_back();
    double Ki = Ks.back(); Ks.pop_back();
    double Kp = Ks.back(); Ks.pop_back();
    
    double error_theta = atan2(setpoint_pose.y - turtle_pose.y, setpoint_pose.x - turtle_pose.x) - turtle_pose.theta;
    double u = Kp*error_theta + Ki*error_theta*dt + Kd*(error_theta - prev_error_theta)/dt;
    prev_error_theta = error_theta;
    return u;
}

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