/*
 * ros_node.cpp
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */

#include "ros_node.h"

RosNode::RosNode(int argc, char **argv, std::string node_name)
{
  ros::init(argc, argv, node_name);
  task_command_ = "";
  gain_p_ = 0;
  gain_i_ = 0;
  gain_d_ = 0;
  test_ = false;
}
RosNode::~RosNode()
{
}
void RosNode::initialize()
{
  ros::NodeHandle nh;

  raw_force_torque_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/sdu/ur10e/raw_force_torque_data", 10);
  filtered_force_torque_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/sdu/ur10e/filtered_force_torque_data", 10);
  pid_compensation_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/sdu/ur10e/pud_compensation_data", 10);

  gazebo_shoulder_pan_position_pub_ = nh.advertise<std_msgs::Float64>("/ur10e_robot/shoulder_pan_position/command", 10);
  gazebo_shoulder_lift_position_pub_ = nh.advertise<std_msgs::Float64>("/ur10e_robot/shoulder_lift_position/command", 10);
  gazebo_elbow_position_pub_ = nh.advertise<std_msgs::Float64>("/ur10e_robot/elbow_position/command", 10);
  gazebo_wrist_1_position_pub_ = nh.advertise<std_msgs::Float64>("/ur10e_robot/wrist_1_position/command", 10);
  gazebo_wrist_2_position_pub_ = nh.advertise<std_msgs::Float64>("/ur10e_robot/wrist_2_position/command", 10);
  gazebo_wrist_3_position_pub_ = nh.advertise<std_msgs::Float64>("/ur10e_robot/wrist_3_position/command", 10);


  command_sub_ = nh.subscribe("/sdu/ur10e/ee_command", 10, &RosNode::CommandDataMsgCallBack, this);
  task_command_sub_ = nh.subscribe("/sdu/ur10e/task_command", 10, &RosNode::TaskCommandDataMsgCallBack, this);
  pid_gain_command_sub_ = nh.subscribe("/sdu/ur10e/pid_gain_command", 10, &RosNode::PidGainCommandMsgCallBack, this);

  test_sub_ =  nh.subscribe("/sdu/ur10e/test", 10, &RosNode::TestMsgCallBack, this);

  set_point_.assign(7,0);
}
void RosNode::CommandDataMsgCallBack (const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if(msg->data[7] <= 0)
    return;

  task_command_ = "set_point";

  for(unsigned int num = 0; num < 7; num ++)
    set_point_[num] = msg->data[num];
}
void RosNode::TaskCommandDataMsgCallBack (const std_msgs::String::ConstPtr& msg)
{
  task_command_ = msg->data;
}
void RosNode::PidGainCommandMsgCallBack (const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  gain_p_ = msg->data[0];
  gain_i_ = msg->data[1];
  gain_d_ = msg->data[2];

  YAML::Emitter y_out;
  std::string path_ = "../config/force_pid_gain.yaml";

  y_out << YAML::BeginMap;
  y_out << YAML::Key << "p_gain";
  y_out << YAML::Value << gain_p_;
  y_out << YAML::Key << "i_gain";
  y_out << YAML::Value << gain_i_;
  y_out << YAML::Key << "d_gain";
  y_out << YAML::Value << gain_d_;
  y_out << YAML::EndMap;
  std::ofstream fout(path_.c_str());
  fout << y_out.c_str(); // dump it back into the file
}
void RosNode::TestMsgCallBack (const std_msgs::Bool::ConstPtr& msg)
{
  test_ = msg->data;
  std::cout << "test!!!!!!!!" << test_ << std::endl;
}
void RosNode::send_gazebo_command (std::vector<double> gazebo_command)
{
  gazebo_shoulder_pan_position_msg_.data = gazebo_command[0];
  gazebo_shoulder_lift_position_msg_.data = gazebo_command[1];
  gazebo_elbow_position_msg_.data = gazebo_command[2];
  gazebo_wrist_1_position_msg_.data = gazebo_command[3];
  gazebo_wrist_2_position_msg_.data = gazebo_command[4];
  gazebo_wrist_3_position_msg_.data = gazebo_command[5];


  gazebo_shoulder_pan_position_pub_.publish(gazebo_shoulder_pan_position_msg_);
  gazebo_shoulder_lift_position_pub_.publish(gazebo_shoulder_lift_position_msg_);
  gazebo_elbow_position_pub_.publish(gazebo_elbow_position_msg_);
  gazebo_wrist_1_position_pub_.publish(gazebo_wrist_1_position_msg_);
  gazebo_wrist_2_position_pub_.publish(gazebo_wrist_2_position_msg_);
  gazebo_wrist_3_position_pub_.publish(gazebo_wrist_3_position_msg_);
}
void RosNode::send_raw_ft_data (std::vector<double> raw_ft_data)
{
  raw_force_torque_msg_.data.push_back(raw_ft_data[0]);
  raw_force_torque_msg_.data.push_back(raw_ft_data[1]);
  raw_force_torque_msg_.data.push_back(raw_ft_data[2]);
  raw_force_torque_msg_.data.push_back(raw_ft_data[3]);
  raw_force_torque_msg_.data.push_back(raw_ft_data[4]);
  raw_force_torque_msg_.data.push_back(raw_ft_data[5]);

  raw_force_torque_pub_.publish(raw_force_torque_msg_);

  raw_force_torque_msg_.data.clear();
}
void RosNode::send_filtered_ft_data (std::vector<double> filtered_ft_data)
{
  filtered_force_torque_msg_.data.push_back(filtered_ft_data[0]);
  filtered_force_torque_msg_.data.push_back(filtered_ft_data[1]);
  filtered_force_torque_msg_.data.push_back(filtered_ft_data[2]);
  filtered_force_torque_msg_.data.push_back(filtered_ft_data[3]);
  filtered_force_torque_msg_.data.push_back(filtered_ft_data[4]);
  filtered_force_torque_msg_.data.push_back(filtered_ft_data[5]);

  filtered_force_torque_pub_.publish(filtered_force_torque_msg_);

  filtered_force_torque_msg_.data.clear();
}
void RosNode::send_pid_compensation_data (std::vector<double> pid_compensation_data)
{
  pid_compensation_msg_.data.push_back(pid_compensation_data[0]);
  pid_compensation_msg_.data.push_back(pid_compensation_data[1]);
  pid_compensation_msg_.data.push_back(pid_compensation_data[2]);
  pid_compensation_msg_.data.push_back(pid_compensation_data[3]);
  pid_compensation_msg_.data.push_back(pid_compensation_data[4]);
  pid_compensation_msg_.data.push_back(pid_compensation_data[5]);

  pid_compensation_pub_.publish(pid_compensation_msg_);

  pid_compensation_msg_.data.clear();
}
void RosNode::update_ros_data()
{
  ros::spinOnce();
}
void RosNode::shout_down_ros()
{
  // mutex lock release
  command_sub_.shutdown();
  task_command_sub_.shutdown();
  pid_gain_command_sub_.shutdown();
  test_sub_.shutdown();
  //

  ros::shutdown();
}
std::string RosNode::get_task_command()
{
  return task_command_;
}
std::vector<double> RosNode::get_set_point()
{
  return set_point_;
}
void RosNode::clear_task_command ()
{
  task_command_ = "";
}
double RosNode::get_p_gain()
{
  return gain_p_;
}
double RosNode::get_i_gain()
{
  return gain_i_;
}
double RosNode::get_d_gain()
{
  return gain_d_;
}
bool RosNode::get_test()
{
  return test_;
}
