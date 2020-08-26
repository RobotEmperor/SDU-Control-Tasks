/*
 * task_motion.h
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */

#ifndef SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_TASK_MOTION_H_
#define SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_TASK_MOTION_H_

#include <Eigen/Dense>
// file load
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>

//yaml
#include <yaml-cpp/yaml.h>

//sdu_math
#include "sdu_math/kinematics.h"
#include "sdu_math/end_point_to_rad_cal.h"
#include <rw/math.hpp>

//log
#include "log.h"

using namespace rw::math;

class TaskMotion
{

public:
  TaskMotion();
  ~TaskMotion();
  void initialize(double control_time_, std::string load_path_);
  void robot_initialize(); // joint space
  void trans_tcp_to_base_motion(std::string load_path_);
  void load_task_motion(std::string path_, std::string motion_);

  bool insert_belt_into_pulley(bool contact_, double change_x, double change_y, double change_z);
  bool up_motion(bool contact_, double x, double y, double z, double axis_x, double axis_y, double axis_z);

  bool close_to_pulleys(double x,double y,double depth);
  bool make_belt_robust(double radious);

  bool finish_1(bool contact_, double x, double y, double z, double axis_x, double axis_y, double axis_z);
  bool finish_2(bool contact_, double x, double y, double z, double axis_x, double axis_y, double axis_z);

  void check_phases();
  void run_task_motion();
  void generate_trajectory();

  bool task_status();

  double calculate_velocity(double first_point,double second_point, double interval_time);
  double calculate_next_velocity(double first_vel, double second_vel);
  void calculate_init_final_velocity(int point_number);

  void clear_task_motion();
  void clear_phase();

  void set_point(double x, double y, double z, double axis_x, double axis_y, double axis_z);
  void set_initial_pose(double x, double y, double z, double axis_x, double axis_y, double axis_z);
  void set_current_pose_eaa(double x, double y, double z, double axis_x, double axis_y, double axis_z);

  void set_pulley_radious(double radious);

  void load_data_initialize();
  void load_data_tcp_motion();

  void change_motion(std::string motion_);
  void stop_motion();

  void tf_set_point_base(std::vector<double> tcp_set_point);

  std::vector<double> get_set_point_base();
  std::vector<double> get_current_pose();
  std::vector<double> get_desired_force_torque();
  std::vector<double> get_initial_ee_position();
  unsigned int get_phases_();

private:
  int number_of_point;
  int all_point;
  int init_all_point;
  int init_belt_task_all_point;
  int tcp_all_point;

  int current_point;
  bool check_change;
  bool task_done;
  bool base_frame_;


  double path_angle_;
  double path_y_;
  double change_path_x_;
  double change_path_y_;
  double change_path_z_;
  unsigned int phases_;
  unsigned int pre_phases_;
  double radious_;

  //initial condition
  //robot's ee position
  std::vector<double> initial_robot_ee_position;
  std::vector<double> bigger_pulley_bearing_position;
  std::vector<double> smaller_pulley_bearing_position;
  std::vector<double> task_initial_position;
  std::vector<double> set_point_;

  //modified robot position (in relative to base)
  std::map<int, std::vector<double>> motion_start_time_vector;
  std::map<int, std::vector<double>> motion_task_pose_vector;
  std::map<int, std::vector<double>> motion_task_init_vel_vector;
  std::map<int, std::vector<double>> motion_task_final_vel_vector;

  //robot initial position (in relative to base)
  std::map<int, std::vector<double>> init_motion_start_time_vector;
  std::map<int, std::vector<double>> init_motion_task_pose_vector;
  std::map<int, std::vector<double>> init_motion_task_init_vel_vector;
  std::map<int, std::vector<double>> init_motion_task_final_vel_vector;

  //init belt (in relative to base)
  std::map<int, std::vector<double>> init_belt_motion_start_time_vector;
  std::map<int, std::vector<double>> init_belt_motion_task_pose_vector;
  std::map<int, std::vector<double>> init_belt_motion_task_init_vel_vector;
  std::map<int, std::vector<double>> init_belt_motion_task_final_vel_vector;

  //tcp (in relative to tcp frame)
  std::map<int, std::vector<double>> tcp_motion_start_time_vector;
  std::map<int, std::vector<double>> tcp_motion_task_pose_vector;

  std::map<int, std::vector<double>> tcp_motion_task_init_vel_vector;
  std::map<int, std::vector<double>> tcp_motion_task_final_vel_vector;

  //tcp force
  std::map<int, std::vector<double>> tcp_motion_desired_force_vector;

  std::vector<double> current_pose_vector;
  std::vector<double> current_force_torque_vector;

  std::shared_ptr<EndEffectorTraj> robot_traj;
  Eigen::MatrixXd desired_pose_matrix;

  Transform3D<> tf_base_to_bearing;
  Transform3D<> tf_base_to_bearing2;
  Transform3D<> tf_bearing_to_init;
  Transform3D<> tf_bearing_to_bearing2;
  Transform3D<> tf_base_to_init_task;
  Transform3D<> tf_static_frame_;
  Transform3D<> tf_contact_point_to_non_contact_point_;


  Transform3D<> tf_tcp_desired_pose_;
  Transform3D<> tf_parts_desired_pose_;
  Transform3D<> tf_current_pose_;
  Transform3D<> tf_desired_pose_;

  Wrench6D<> tf_force_desired_;
  Wrench6D<> tf_tcp_desired_force_;
};
#endif /* TASK_MOTION_H_ */

