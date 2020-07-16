/*
 * ur_robot.h
 *
 *  Created on: Jul 16, 2020
 *      Author: yik
 */

#ifndef SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_UR_ROBOT_H_
#define SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_UR_ROBOT_H_
#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <Eigen/Dense>
#include <stdio.h>
#include "log.h"

//yaml
#include <yaml-cpp/yaml.h>

//ur_rtde library
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>

//rw_robwork
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/kinematics.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/math.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include "sdu_math/end_point_to_rad_cal.h"
#include "sdu_math/control_function.h"
#include "sdu_math/statistics_math.h"
#include "task_motion.h"
#include "tool_estimation.h"

using namespace ur_rtde;
using namespace rw::math;
using namespace rw::models;
using namespace std;
using rw::invkin::ClosedFormIKSolverUR;
using rw::kinematics::State;
using rw::loaders::WorkCellLoader;

#define WC_FILE "/home/yik/sdu_ws/SDU-Control-Tasks/wc/UR10e_2018/UR10e.xml"


class UrRobot
{
public:
  UrRobot(double init_control_time);
  ~UrRobot();

  void initialize(std::string robot_ip_); // real robot
  void load_initialize_parameter(std::string path_);
  void load_task_motion(std::string path_, std::string task_);
  void simulation_initialize();// simulation
  void init_accelerometer();
  void controller();
  void robot_servo_stop();

  //set data
  void set_current_task_motion(std::string command_);
  void set_position_gain(double p_, double i_, double d_);
  void set_force_gain(double p_, double i_, double d_);
  void set_test_contact(bool check_);

  //get data
  double get_time_count();
  std::vector<double> get_raw_ft_data();
  std::vector<double> get_actual_joint_positions();
  std::vector<double> get_actual_tcp_pose();
  std::vector<double> get_target_tcp_pose();
  std::vector<double> get_acutal_tcp_acc();
  std::vector<double> get_pid_compensation();
  std::vector<double> get_desired_pose_vector();
  std::vector<double> get_filtered_tcp_ft_data();
  std::vector<double> get_contacted_ft_data();
  std::vector<double> get_desired_q_();


private:
  bool gazebo_check_;

  //control
  double control_time_;
  double time_count_;
  double motion_time_;
  double f_kp_;
  double f_ki_;
  double f_kd_;

  double p_kp_;
  double p_ki_;
  double p_kd_;

  bool contact_check_;

  //solution check
  bool joint_vel_limits_;

  std::vector<Q> solutions_;
  int solution_number_;

  State state_;
  std::shared_ptr<ClosedFormIKSolverUR> solver_;

  //robot interfaces
  std::shared_ptr<RTDEReceiveInterface> rtde_receive;
  std::shared_ptr<RTDEControlInterface> rtde_control;

  //traj and task motion
  std::shared_ptr<EndEffectorTraj> ur10e_traj;
  std::shared_ptr<TaskMotion> ur10e_task;

  //tool estimation
  std::shared_ptr<ToolEstimation> tool_estimation;
  double mass_;

  //cusum_method
  std::shared_ptr<StatisticsMath> statistics_math;

  //pid controller
  std::shared_ptr<PID_function> force_x_compensator;
  std::shared_ptr<PID_function> force_y_compensator;
  std::shared_ptr<PID_function> force_z_compensator;

  std::shared_ptr<PID_function> position_x_controller;
  std::shared_ptr<PID_function> position_y_controller;
  std::shared_ptr<PID_function> position_z_controller;

  //robot states
  std::vector<double> actual_joint_positions_;//(6);
  std::vector<double> actual_tcp_pose_;//(6);
  std::vector<double> target_tcp_pose_;//(6);
  std::vector<double> acutal_tcp_acc_;//(3);
  std::vector<double> raw_ft_data_;//(6);A
  std::vector<double> filtered_tcp_ft_data_;//(6);
  std::vector<double> contacted_ft_data_;//(6);
  std::vector<double> contacted_ft_no_offset_data_;//(6);
  std::vector<double> current_q_;//(6);
  std::vector<double> pid_compensation_;//(6);

  //control states
  std::vector<double> set_point_vector_;//(6);
  std::vector<double> desired_pose_vector_;//(6);
  std::vector<double> desired_force_torque_vector_;//(6);
  std::vector<double> compensated_pose_vector_;//(6);

  //task motion
  std::string current_task_command_;
  std::string previous_task_command_;

  //tf
  rw::math::Transform3D<> tf_tcp_desired_pose_;
  rw::math::Transform3D<> tf_modified_pose_;
  rw::math::Transform3D<> tf_current_;
  rw::math::Transform3D<> tf_desired_;

  rw::math::Wrench6D<> current_ft_;
  rw::math::Wrench6D<> current_ft_no_offset_;
  rw::math::Wrench6D<> tf_tcp_current_force_;

};
#endif /* SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_UR_ROBOT_H_ */
