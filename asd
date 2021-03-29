#include "aubo_driver/aubo_driver.h"
#include "aubo_driver.cpp"
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <aubo_msgs/JointPos.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include "tf/tf.h"
#include "geometry_msgs/PoseArray.h"

#include <string>
#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <sstream>
#include <fstream>

using namespace aubo_driver;
#define MAX_JOINT_ACC 100.0/180.0*M_PI  //unit rad/s^2
#define MAX_JOINT_VEL 20.0/180.0*M_PI   //unit rad/s
#define MAX_END_ACC    4                // unit m/s^2
#define MAX_END_VEL    0.02                // unit m/s
class ROSNode
{
public:
  ROSNode(ros::NodeHandle &_n) {
    n = &_n;
  };
  ros::NodeHandle *n;
};
ROSNode *rn;

sensor_msgs::JointState js;
geometry_msgs::PoseArray pp;

double currentjoint[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; //degree
double  targetjoint[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; //degree
std::vector<double> fromik ={0.0,0.0,0.0,0.0,0.0,0.0};
std::vector<double> rpy = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double roll,pitch,yaw;
int state = 0;

std::vector<double> GetIK(geometry_msgs::Pose &_ps);
geometry_msgs::Pose jsCartesian(const sensor_msgs::JointState &_js, std::vector<double> &_pose);
 
void jointCallback(const sensor_msgs::JointState msg){
     ROS_INFO("%f,%f,%f,%f,%f,%f", msg.position[0],msg.position[1],msg.position[2],msg.position[3],msg.position[4],msg.position[5]);
     for(int i=0; i<6; i++) {
        currentjoint[i] = msg.position[i];
             
    }
}
void joyCallback(const sensor_msgs::Joy msg){
    ROS_INFO("joyCallback : %s", msg.header.frame_id.c_str());

    //ปุ่มอนาล็อคซ้าย ขยับซ้าย << หมายถึงปุ่มไม่ใช่หุ่น (ขยับฐาน)
    if(msg.axes[0] > 0.25 && state == 0){
        state = 1;
    }
     //ปุ่มอนาล็อคซ้าย ขยับขวา ขยับฐาน
    else if (msg.axes[0] < -0.25 && state == 0){
        state = 2;
    }
    //ปุ่มอนาล็อคซ้ายขยับขึ้น joint2 ขยับขึ้น
    else if (msg.axes[1] > 0.25 && state == 0){

        state = 3;    
    }
    //ปุ่มอนาล็อคซ้ายขยับลง joint2 ขยับลง
    else if (msg.axes[1] < -0.25 && state == 0){

        state = 4; 
    }
    //ปุ่มอนาล็อคขวาขยับขึ้น joint3 ขยับขึ้น
    else if  (msg.axes[4] > 0.25 && state == 0){

        state = 5;
    }
    //ปุ่มอนาล็อคขวาขยับลง joint3 ขยับลง
    else if (msg.axes[4] < -0.25 && state == 0){

        state = 6;
    }
    //ปุ่มอนาล็อคขวาขยับ ขวา joint5 ขยับขวา
    else if (msg.axes[3] > 0.25 && state == 0){

        state = 7;    
    }
    //ปุ่มอนาล็อคขวาขยับ ซ้าย joint 5 ขยับซ้าย
    else if (msg.axes[3] < -0.25 && state == 0){

        state = 8;
    }
    //ปุ่มลูกศรขึ้นjoint4 ขยับขึ้น
    else if (msg.axes[7] == 1 && state == 0){

        state = 9;    
    }
    //ปุ่มลูกศรลงjoint4ขยับลง
    else if (msg.axes[7] == -1 && state == 0){

        state = 10;    
    }
    //ปุ่มลูกศรซ้าย joint6 ขยับซ้าย
    else if (msg.axes[6] == 1 && state == 0){

        state = 11;
    }
    //ปุ่มลูกศรขวา joint6ขยับขวา
    else if (msg.axes[6] == -1 && state == 0){

        state = 12;
    }
    else if (msg.buttons[0] == 1 && state ==0){
        state = 13;
    }
    else{
        state = 0;
    }

}


int main(int argc, char **argv)
{
 
  
  
  ros::init(argc, argv, "testAuboAPI");
  ros::NodeHandle nn;
  rn= new ROSNode(nn);



  ros::Subscriber joy_sub = rn->n->subscribe("joy", 1000, joyCallback );   //subscribe joy
  ros::Subscriber jointState = rn->n->subscribe("/joint_states",1000, jointCallback); //subscribe jointstate
 // ros::ServiceClient srv_fk = n.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
//  ros::ServiceClient srv_ik = n.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
  


  AuboDriver robot_driver;
  bool ret = robot_driver.connectToRobotController();
  


  /** If connect to a real robot, then you need initialize the dynamics parameters　**/
  aubo_robot_namespace::ROBOT_SERVICE_STATE result;
  //tool parameters
  aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
  memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

  robot_driver.robot_send_service_.rootServiceRobotStartup(toolDynamicsParam/**tool dynamics paramters**/,
                                             6        /*collision class*/,
                                             true     /* Is allowed to read robot pose*/,
                                             true,    /*default */
                                             1000,    /*default */
                                             result); /*initialize*/

  ros::Rate loop_rate(1000);
  while (ros::ok()){
  	  if(ret && state == 1){
			ROS_INFO("state 1");//move left ปุ่มอนาล็อคซ้าย ขยับซ้าย
		   geometry_msgs::Pose cartesian = jsCartesian(js, rpy);
			cartesian.position.x-0.1; 
			GetIK(cartesian);
		
      	fromik = GetIK(cartesian);		
			for(int i=0;i<6;i++){
				targetjoint[i] = fromik[i];
			}
      	robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);
		}
      else if(ret && state == 2)
      {
			ROS_INFO("state 2");//move right ปุ่มอนาล็อคซ้าย ขยับขวา 
      	geometry_msgs::Pose cartesian = jsCartesian(js, rpy);
			cartesian.position.x+0.1;
			GetIK(cartesian);
      	fromik = GetIK(cartesian);		
			for(int i=0;i<6;i++){
				targetjoint[i] = fromik[i];
			}
        	robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);
      }
      else if(ret && state == 3)
      {
			ROS_INFO("state 3"); //move up ปุ่มอนาล็อคซ้ายขยับขึ้น
      	geometry_msgs::Pose cartesian = jsCartesian(js, rpy);
			cartesian.position.z+0.1;
			GetIK(cartesian);
      	fromik = GetIK(cartesian);		
			for(int i=0;i<6;i++){
				targetjoint[i] = fromik[i];
			}
        	robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);
      }
      else if(ret && state == 4)
      {
			ROS_INFO("state 4"); //move down ปุ่มอนาล็อคซ้ายขยับลง 
      	geometry_msgs::Pose cartesian = jsCartesian(js, rpy);
			cartesian.position.z-0.1;
			GetIK(cartesian);
      	fromik = GetIK(cartesian);		
			for(int i=0;i<6;i++){
				targetjoint[i] = fromik[i];
			}
        	robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);
      }	
      else if(ret && state == 5)
      {
			ROS_INFO("state 5"); //move backward (maybe) ปุ่มอนาล็อคขวาขยับขึ้น
      	geometry_msgs::Pose cartesian = jsCartesian(js, rpy);
			cartesian.position.y-0.1;
			GetIK(cartesian);
      	fromik = GetIK(cartesian);		
			for(int i=0;i<6;i++){
				targetjoint[i] = fromik[i];
			}
        	robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);
      }
      else if(ret && state == 6)
      {
			ROS_INFO("state 6"); //move forward (maybe) ปุ่มอนาล็อคขวาขยับลง
      	geometry_msgs::Pose cartesian = jsCartesian(js, rpy);
			cartesian.position.y+0.1;
			GetIK(cartesian);
      	fromik = GetIK(cartesian);		
			for(int i=0;i<6;i++){
				targetjoint[i] = fromik[i];
			}
        	robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);
      }
      else if(ret && state == 11)
      {
			ROS_INFO("state 11"); //yaw (rotate z) D-pad left
      	geometry_msgs::Pose cartesian = jsCartesian(js, rpy);
    		tf::Quaternion qtorpy;
    		tf::quaternionMsgToTF(cartesian.orientation , qtorpy);
    		tf::Matrix3x3 m(qtorpy);
			m.getRPY(roll, pitch, yaw);
			yaw-0.1;
			tf::Quaternion rpytoq;
			rpytoq.setRPY(roll,pitch,yaw);
			rpytoq = rpytoq.normalize();
						
			GetIK(cartesian);
      	fromik = GetIK(cartesian);		
			for(int i=0;i<6;i++){
				targetjoint[i] = fromik[i];
			}
        	robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);
      }
      else if(ret && state == 12)
      {
			ROS_INFO("state 12"); //yaw (rotate z) D-pad right
      	geometry_msgs::Pose cartesian = jsCartesian(js, rpy);
    		tf::Quaternion qtorpy;
    		tf::quaternionMsgToTF(cartesian.orientation , qtorpy);
    		tf::Matrix3x3 m(qtorpy);
			m.getRPY(roll, pitch, yaw);
			yaw+0.1;
			tf::Quaternion rpytoq;
			rpytoq.setRPY(roll,pitch,yaw);
			rpytoq = rpytoq.normalize();
						
			GetIK(cartesian);
      	fromik = GetIK(cartesian);		
			for(int i=0;i<6;i++){
				targetjoint[i] = fromik[i];
			}
        	robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);
      }
      else if(ret && state == 7)
      {
			ROS_INFO("state 7"); //roll (rotate x) right analog go right
      	geometry_msgs::Pose cartesian = jsCartesian(js, rpy);
    		tf::Quaternion qtorpy;
    		tf::quaternionMsgToTF(cartesian.orientation , qtorpy);
    		tf::Matrix3x3 m(qtorpy);
			m.getRPY(roll, pitch, yaw);
			roll+0.1;
			tf::Quaternion rpytoq;
			rpytoq.setRPY(roll,pitch,yaw);
			rpytoq = rpytoq.normalize();
						
			GetIK(cartesian);
      	fromik = GetIK(cartesian);		
			for(int i=0;i<6;i++){
				targetjoint[i] = fromik[i];
			}
        	robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);
      }
      else if(ret && state == 8)
      {
			ROS_INFO("state 8"); //roll (rotate x) right analog go left
      	geometry_msgs::Pose cartesian = jsCartesian(js, rpy);
    		tf::Quaternion qtorpy;
    		tf::quaternionMsgToTF(cartesian.orientation , qtorpy);
    		tf::Matrix3x3 m(qtorpy);
			m.getRPY(roll, pitch, yaw);
			roll-0.1;
			tf::Quaternion rpytoq;
			rpytoq.setRPY(roll,pitch,yaw);
			rpytoq = rpytoq.normalize();
						
			GetIK(cartesian);
      	fromik = GetIK(cartesian);		
			for(int i=0;i<6;i++){
				targetjoint[i] = fromik[i];
			}
        	robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);
      }
      else if(ret && state == 9)
      {
			ROS_INFO("state 9"); //pitch (rotate y) ปุ่มลูกศรขึ้น
      	geometry_msgs::Pose cartesian = jsCartesian(js, rpy);
    		tf::Quaternion qtorpy;
    		tf::quaternionMsgToTF(cartesian.orientation , qtorpy);
    		tf::Matrix3x3 m(qtorpy);
			m.getRPY(roll, pitch, yaw);
			pitch-0.1;
			tf::Quaternion rpytoq;
			rpytoq.setRPY(roll,pitch,yaw);
			rpytoq = rpytoq.normalize();
						
			GetIK(cartesian);
      	fromik = GetIK(cartesian);		
			for(int i=0;i<6;i++){
				targetjoint[i] = fromik[i];
			}
        	robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);
      }
      else if(ret && state == 10)
      {
			ROS_INFO("state 10"); //pitch (rotate y) ปุ่มลูกศรลง
      	geometry_msgs::Pose cartesian = jsCartesian(js, rpy);
    		tf::Quaternion qtorpy;
    		tf::quaternionMsgToTF(cartesian.orientation , qtorpy);
    		tf::Matrix3x3 m(qtorpy);
			m.getRPY(roll, pitch, yaw);
			pitch+0.1;
			tf::Quaternion rpytoq;
			rpytoq.setRPY(roll,pitch,yaw);
			rpytoq = rpytoq.normalize();
			
						
			GetIK(cartesian);
      	fromik = GetIK(cartesian);		
			for(int i=0;i<6;i++){
				targetjoint[i] = fromik[i];
			}
        	robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);
      }

	

   
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
std::vector<double> GetIK(geometry_msgs::Pose &_ps){
	    
      moveit_msgs::GetPositionIK msg;
      msg.request.ik_request.group_name = "manipulator_i5";
      msg.request.ik_request.pose_stamped.pose = _ps;
      msg.request.ik_request.attempts = 0;
      ros::ServiceClient srv_ik = rn->n->serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
      if(srv_ik.call(msg)){
          ROS_INFO("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",msg.response.solution.joint_state.position[0]*180/M_PI,msg.response.solution.joint_state.position[1]*180/M_PI,msg.response.solution.joint_state.position[2]*180/M_PI,msg.response.solution.joint_state.position[3]*180/M_PI,msg.response.solution.joint_state.position[4]*180/M_PI,msg.response.solution.joint_state.position[5]*180/M_PI);
	  
      }
      else{
        ROS_ERROR("Failed to call srv ");
      }
	  return msg.response.solution.joint_state.position;
  }
geometry_msgs::Pose jsCartesian(const sensor_msgs::JointState &_js, std::vector<double> &_pose){
  moveit_msgs::GetPositionFK msg;
  msg.request.header.stamp = ros::Time::now();
  msg.request.fk_link_names = {"wrist3_Link"};
  msg.request.robot_state.joint_state = _js;
  ros::ServiceClient srv_fk = rn->n->serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
  if(srv_fk.call(msg)){
    _pose[0] = msg.response.pose_stamped[0].pose.position.x;
    _pose[1] = msg.response.pose_stamped[0].pose.position.y;
    _pose[2] = msg.response.pose_stamped[0].pose.position.z; 
    tf::Quaternion q_ori;
    tf::quaternionMsgToTF(msg.response.pose_stamped[0].pose.orientation , q_ori);
    tf::Matrix3x3 m(q_ori);
    double r, p, y;
    m.getRPY(r, p, y);
    _pose[3] = r;
    _pose[4] = p;
    _pose[5] = y;
  }
  else{
    ROS_ERROR("Failed to call srv");

  }
  return msg.response.pose_stamped[0].pose;
}
