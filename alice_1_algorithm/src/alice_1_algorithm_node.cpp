/*
 * alice_1_algorithm_node.cpp
 *
 *      Author: arirang2067
 */
#include <ros/ros.h>
#include <cmath>
#include <std_msgs/Float64.h>
#define PI 3.141592

using namespace std;
double Degree_To_Radian(double degree);
double Radian_To_Degree(double radian);

class Alice_1_Joint
{
private:

public:
  int joint_id;
  double current_radian;
  double desired_radian;
  double radian_angular_velocity;
  double radian_threshold;
  bool input_flag;
  bool desired_flag;

  ros::Publisher joint_command;
  std_msgs::Float64 joint_command_msg;

  Alice_1_Joint()
  {

    joint_id = 0;
    current_radian = 0;
    desired_radian = 0;
    radian_angular_velocity = 0;
    radian_threshold = 0.0001;
    input_flag = false;
    desired_flag = false;
  }
  ~Alice_1_Joint(){}

  void Set_Joint_ID(int num,ros::NodeHandle &nh)
  {
    joint_id = num;
  	switch(joint_id)
  	{
      case 1: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/l_shoulder_pitch_position/command", 10); break;
      case 2: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/r_shoulder_pitch_position/command", 10); break;
      case 3: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/l_shoulder_roll_position/command", 10); break;
      case 4: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/r_shoulder_roll_position/command", 10); break;
      case 5: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/l_elbow_pitch_position/command", 10); break;
      case 6: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/r_elbow_pitch_position/command", 10); break;
      case 7: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/head_pitch_position/command", 10); break;
      case 8: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/head_yaw_position/command", 10); break;
      case 11: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/l_hip_pitch_position/command", 10); break;
      case 12: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/r_hip_pitch_position/command", 10); break;
      case 13: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/l_hip_roll_position/command", 10); break;
      case 14: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/r_hip_roll_position/command", 10); break;
      case 15: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/l_hip_yaw_position/command", 10); break;
      case 16: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/r_hip_yaw_position/command", 10); break;
      case 17: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/l_knee_pitch_position/command", 10); break;
      case 18: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/r_knee_pitch_position/command", 10); break;
      case 19: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/l_ankle_pitch_position/command", 10); break;
      case 20: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/r_ankle_pitch_position/command", 10); break;
      case 21: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/l_ankle_roll_position/command", 10); break;
      case 22: joint_command = nh.advertise<std_msgs::Float64>("/alice_1_robot/r_ankle_roll_position/command", 10); break;
    }
    ROS_INFO("Joint %d is initialized!", joint_id);
  }


  void Joint_Controller(double desired_degree, double degree_angular_velocity)
  {
	if(input_flag == false)
	{
      desired_radian = Degree_To_Radian(desired_degree);
      radian_angular_velocity = Degree_To_Radian(degree_angular_velocity);
      input_flag = true;
	}

	if(desired_flag == false)
	{
	  if(desired_radian > current_radian)
      {
        current_radian = current_radian + radian_angular_velocity;
      }
	  else if(desired_radian < current_radian)
      {
        current_radian = current_radian - radian_angular_velocity;
      }
	  else;

	  if(abs(desired_radian - current_radian) <= radian_angular_velocity)
	  {
	    if(abs(desired_radian - current_radian) <= radian_threshold)
	    {
		  desired_flag = true;
	    }
	    else radian_angular_velocity = radian_angular_velocity/2;
	  }
	}
	joint_command_msg.data = current_radian;
	joint_command.publish(joint_command_msg);
	ROS_INFO("Joint%d Degree : %f ", joint_id, Radian_To_Degree(current_radian));
  }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////
double Degree_To_Radian(double degree)
{
  return degree*(PI/180);
}
double Radian_To_Degree(double radian)
{
  return radian*(180/PI);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "alice_1_algorithm_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  vector <Alice_1_Joint> joint(22+1);
  for(int i=1;i<23;i++)
  {
    joint[i].Set_Joint_ID(i,n);
  }

  while(ros::ok())
  {
	joint[1].Joint_Controller(60, 1);
	joint[2].Joint_Controller(60, 1);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
