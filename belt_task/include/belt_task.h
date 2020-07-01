/*
 * belt_task.h
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */
#ifndef SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_BELT_TASK_H_
#define SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_BELT_TASK_H_

#include <Eigen/Dense>
#include "log.h"

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

//xenomai rt system
#include <unistd.h>
#include <signal.h>
#include <cstdlib>
#include <sys/mman.h>
#include <sys/types.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>

#include "sdu_math/end_point_to_rad_cal.h"
#include "sdu_math/control_function.h"
#include "sdu_math/statistics_math.h"
#include "task_motion.h"
#include "ros_node.h"
#include "data_logging.h"
#include "tool_estimation.h"

#include <signal.h> //  our new library

#define WC_FILE "/home/yik/sdu_ws/SDU-Control-Tasks/wc/UR10e_2018/UR10e.xml"
#define CLOCK_RES 1e-9 //Clock resolution is 1 us by default 1e-9
#define LOOP_PERIOD 2e6 //Expressed in ticks // 2ms control time
//RTIME period = 1000000000;
RT_TASK loop_task;
//RT_TASK loop_task_b;
//RT_TASK loop_task_c;

using namespace std;
using namespace ur_rtde;
using namespace rw::math;
using namespace rw::models;
using rw::invkin::ClosedFormIKSolverUR;
using rw::kinematics::State;
using rw::loaders::WorkCellLoader;

void initialize();

std::string robot_ip_a;
std::string robot_ip_b;

//ros
std::shared_ptr<RosNode> ros_state;
std::shared_ptr<DataLogging> data_log;

bool gazebo_check;
bool exit_program;

//model definition
WorkCell::Ptr wc;
SerialDevice::Ptr device;

std::string silmulation_on_off;

//control
double control_time;
double time_count;
double motion_time;
double f_kp;
double f_ki;
double f_kd;

bool contact_check;

//solution check
bool joint_vel_limits;
std::vector<Q> solutions;

//traj and task motion
std::shared_ptr<EndEffectorTraj> ur10e_traj;
std::shared_ptr<TaskMotion> ur10e_task;

//tool estimation
std::shared_ptr<ToolEstimation> tool_estimation;

//cusum_method
std::shared_ptr<StatisticsMath> statistics_math;

//pid controller
std::shared_ptr<PID_function> force_x_compensator;
std::shared_ptr<PID_function> force_y_compensator;
std::shared_ptr<PID_function> force_z_compensator;

//robot interface
std::shared_ptr<RTDEReceiveInterface> rtde_receive_a;
std::shared_ptr<RTDEControlInterface> rtde_control_a;

std::shared_ptr<RTDEReceiveInterface> rtde_receive_b;
std::shared_ptr<RTDEControlInterface> rtde_control_b;

//robot states
std::vector<double> joint_positions(6);
std::vector<double> actual_tcp_pose(6);
std::vector<double> target_tcp_pose(6);
std::vector<double> acutal_tcp_acc(3);
std::vector<double> raw_ft_data(6);
std::vector<double> filtered_tcp_ft_data(6);
std::vector<double> contacted_ft_data(6);
std::vector<double> current_q(6);
std::vector<double> pid_compensation(6);

//control states
std::vector<double> set_point_vector(6);
std::vector<double> desired_pose_vector(6);
std::vector<double> desired_force_torque_vector(6);
std::vector<double> compensated_pose_vector(6);

//task motion
std::string previous_task_command;

//tf
rw::math::Transform3D<> tf_tcp_desired_pose;
rw::math::Transform3D<> tf_modified_pose;
rw::math::Transform3D<> tf_current;
rw::math::Transform3D<> tf_desired;

rw::math::Wrench6D<> current_ft;
rw::math::Wrench6D<> tf_tcp_current_force;


#endif /* SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_BELT_TASK_H_ */
