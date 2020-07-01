/*
 * ros_node.h
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */

#ifndef SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_ROS_NODE_H_
#define SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_ROS_NODE_H_

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

// file load
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>


#include <yaml-cpp/yaml.h>

class RosNode
{
public:
  RosNode(int argc, char **argv, std::string node_name);
  ~RosNode();

  void initialize();
  void update_ros_data();
  void shout_down_ros();

  void CommandDataMsgCallBack (const std_msgs::Float64MultiArray::ConstPtr& msg);
  void TaskCommandDataMsgCallBack (const std_msgs::String::ConstPtr& msg);
  void PidGainCommandMsgCallBack (const std_msgs::Float64MultiArray::ConstPtr& msg);
  void send_gazebo_command (std::vector<double> gazebo_command);
  void send_raw_ft_data (std::vector<double> raw_ft_data);
  void send_filtered_ft_data (std::vector<double> filtered_ft_data);
  void send_pid_compensation_data (std::vector<double> pid_compensation_data);

  void clear_task_command ();

  void TestMsgCallBack (const std_msgs::Bool::ConstPtr& msg);

  std::vector<double> get_set_point();
  double get_p_gain();
  double get_i_gain();
  double get_d_gain();

  bool get_test();

  std::string get_task_command();

private:

  double gain_p_, gain_i_, gain_d_;
  std::string task_command_;

  std::vector<double> set_point_;

  ros::Publisher raw_force_torque_pub_;
  ros::Publisher filtered_force_torque_pub_;
  ros::Publisher pid_compensation_pub_;
  ros::Publisher gazebo_shoulder_pan_position_pub_;
  ros::Publisher gazebo_shoulder_lift_position_pub_;
  ros::Publisher gazebo_elbow_position_pub_;
  ros::Publisher gazebo_wrist_1_position_pub_;
  ros::Publisher gazebo_wrist_2_position_pub_;
  ros::Publisher gazebo_wrist_3_position_pub_;

  ros::Subscriber command_sub_;
  ros::Subscriber task_command_sub_;
  ros::Subscriber pid_gain_command_sub_;

  ros::Subscriber test_sub_;

  std_msgs::Float64MultiArray raw_force_torque_msg_;
  std_msgs::Float64MultiArray filtered_force_torque_msg_;
  std_msgs::Float64MultiArray pid_compensation_msg_;

  std_msgs::Float64 gazebo_shoulder_pan_position_msg_;
  std_msgs::Float64 gazebo_shoulder_lift_position_msg_;
  std_msgs::Float64 gazebo_elbow_position_msg_;
  std_msgs::Float64 gazebo_wrist_1_position_msg_;
  std_msgs::Float64 gazebo_wrist_2_position_msg_;
  std_msgs::Float64 gazebo_wrist_3_position_msg_;

  //test
  bool test_;
};



#endif /* SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_ROS_NODE_H_ */
