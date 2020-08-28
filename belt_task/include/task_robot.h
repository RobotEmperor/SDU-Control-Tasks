/*
 * task_robot.h
 *
 *  Created on: Aug 19, 2020
 *      Author: yik
 */

#ifndef SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_TASK_ROBOT_H_
#define SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_TASK_ROBOT_H_
//#define WC_FILE "/home/yik/sdu_ws/SDU-Control-Tasks/belt_task/config/wc/UR10e_2018/UR10e.xml"

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

#include "tool_estimation.h"
#include "sdu_math/end_point_to_rad_cal.h"
#include "sdu_math/control_function.h"
#include "sdu_math/statistics_math.h"
#include "task_motion.h"
#include "data_logging.h"

#include <unistd.h>

using namespace ur_rtde;
using namespace rw::models;
using rw::invkin::ClosedFormIKSolverUR;
using rw::kinematics::State;
using rw::loaders::WorkCellLoader;

class TaskRobot
{
public:
	TaskRobot();
	TaskRobot(std::string robot_name, std::string init_path);
	~TaskRobot();

	void parse_init_data_(const std::string &path);
	void initialize(std::string robot_ip, bool gazebo_check);
	void init_model(std::string wc_file, std::string robot_model);
	void move_to_init_pose();

	void estimation_of_belt_position(double radious);
	bool tasks(std::string command);
	bool hybrid_controller();

	void terminate_robot();
	void terminate_data_log();

	void set_force_controller_x_gain(double kp,double ki,double kd);
	void set_force_controller_y_gain(double kp,double ki,double kd);
	void set_force_controller_z_gain(double kp,double ki,double kd);

	void set_force_controller_eaa_x_gain(double kp,double ki,double kd);
	void set_force_controller_eaa_y_gain(double kp,double ki,double kd);
	void set_force_controller_eaa_z_gain(double kp,double ki,double kd);

	void set_position_controller_x_gain(double kp,double ki,double kd);
	void set_position_controller_y_gain(double kp,double ki,double kd);
	void set_position_controller_z_gain(double kp,double ki,double kd);

	void set_position_controller_eaa_x_gain(double kp,double ki,double kd);
	void set_position_controller_eaa_y_gain(double kp,double ki,double kd);
	void set_position_controller_eaa_z_gain(double kp,double ki,double kd);

	void set_robust_value(double robust_value);
	void set_belt_change_values(double x, double y, double z);
	void set_tf_static_robot(rw::math::Transform3D<> tf_base_to_staric_robot, rw::math::Transform3D<> tf_base_to_bearing_staric_robot);

	std::vector<double> get_raw_ft_data_();
	std::vector<double> get_contacted_ft_data_();
	std::vector<double> get_error_ee_pose_();
	std::vector<double> get_actual_tcp_speed_();
	std::vector<double> get_current_q_();
	rw::math::Transform3D<> get_tf_current_();
	rw::math::Transform3D<> get_tf_base_to_bearing_();

private:

	struct PIDControllerGain
	{
		double x_kp;
		double x_ki;
		double x_kd;

		double y_kp;
		double y_ki;
		double y_kd;

		double z_kp;
		double z_ki;
		double z_kd;

		double eaa_x_kp;
		double eaa_x_ki;
		double eaa_x_kd;

		double eaa_y_kp;
		double eaa_y_ki;
		double eaa_y_kd;

		double eaa_z_kp;
		double eaa_z_ki;
		double eaa_z_kd;
	};

	PIDControllerGain force_controller_gain_;
	PIDControllerGain position_controller_gain_;

	State state_;

	std::string robot_name_;

	//robot interface
	std::shared_ptr<RTDEReceiveInterface> rtde_receive_;
	std::shared_ptr<RTDEControlInterface> rtde_control_;

	//traj and task motion
	std::shared_ptr<EndEffectorTraj> robot_fifth_traj_;
	std::shared_ptr<TaskMotion> robot_task_;

	//tool estimation
	std::shared_ptr<ToolEstimation> tool_estimation_;

	//cusum_method
	std::shared_ptr<StatisticsMath> statistics_math_;

	//pid controller
	std::shared_ptr<PID_function> force_x_compensator_;
	std::shared_ptr<PID_function> force_y_compensator_;
	std::shared_ptr<PID_function> force_z_compensator_;

	std::shared_ptr<PID_function> position_x_controller_;
	std::shared_ptr<PID_function> position_y_controller_;
	std::shared_ptr<PID_function> position_z_controller_;

	//data logging
	std::shared_ptr<DataLogging> data_log_;

	//robot states
	std::vector<double> joint_positions_; //(6);
	std::vector<double> actual_tcp_pose_; //(6);
	std::vector<double> actual_tcp_speed_; //(6);
	std::vector<double> target_tcp_pose_; //(6);
	std::vector<double> acutal_tcp_acc_; //(3);
	std::vector<double> raw_ft_data_; //(6);
	std::vector<double> filtered_tcp_ft_data_; //(6);
	std::vector<double> contacted_ft_data_; //(6);
	std::vector<double> contacted_ft_no_offset_data_; //(6);
	std::vector<double> current_q_; //(6);
	std::vector<double> pid_compensation_; //(6);

	//control states
	std::vector<double> set_point_vector_; //(6);
	std::vector<double> desired_pose_vector_; //(6);
	std::vector<double> desired_force_torque_vector_; //(6);
	std::vector<double> compensated_pose_vector_; //(6);
	std::vector<double> error_ee_pose_; //(6);

	std::vector<double> compensated_q_; //(6);

	//controller gains
	std::vector<double> force_controller_; //(6);
	std::vector<double> position_controller_; //(6);

  std::vector<double> data_current_belt_;
  std::vector<double> data_desired_belt_;

	//task motion
	std::string previous_task_command_;

	//tf
	rw::math::Transform3D<> tf_tcp_desired_pose_;
	rw::math::Transform3D<> tf_modified_pose_;
	rw::math::Transform3D<> tf_current_;
	rw::math::Transform3D<> tf_desired_;
	rw::math::Transform3D<> tf_base_to_bearing_;

	rw::math::Wrench6D<> current_ft_;
	rw::math::Wrench6D<> current_ft_no_offset_;
	rw::math::Wrench6D<> tf_tcp_current_force_;
	rw::math::Transform3D<> tf_base_to_static_robot_;
	rw::math::Transform3D<> tf_base_to_bearing_static_robot_;

	//control
	double control_time_;
	double time_count_;
	double tool_mass_;
	double belt_robust_value_;
	double change_x_, change_y_, change_z_;
	double current_belt_x_, current_belt_y_, current_belt_z_;
	double desired_belt_x_, desired_belt_y_, desired_belt_z_;

	bool contact_check_;
	bool control_check_;
	bool previous_phase_;

	//solution check
	int preferred_solution_number_;
	bool joint_vel_limits_;
	std::vector<rw::math::Q> solutions_;

	//model identification
	std::string wc_file_;
	WorkCell::Ptr wc_;
	SerialDevice::Ptr device_;
	std::shared_ptr<ClosedFormIKSolverUR> solver_;

	// gazebo
	bool gazebo_check_;

	//
	bool flag;

	rw::math::Transform3D<> tf_bearing_to_static_robot;
	rw::math::Transform3D<> tf_bearing_to_rubber_point;
	rw::math::Transform3D<> tf_bearing_to_moveable_robot_start;
	rw::math::Transform3D<> tf_bearing_to_moveable_robot;
	rw::math::Transform3D<> tf_bearing_to_error;
	rw::math::Transform3D<> distance_tf_static_robot_to_moveable_robot;
	rw::math::Transform3D<> distance_tf_static_robot_to_moveable_robot_start;
	rw::math::Transform3D<> tf_static_robot_to_bearing;
	rw::math::Transform3D<> tf_static_robot_to_rubber_point;
	rw::math::Transform3D<> tf_moveable_robot_to_bearing;
  rw::math::Transform3D<> tf_moveable_robot_to_init_belt_;
	rw::math::Transform3D<> tf_moveable_robot_to_error;

	rw::math::Transform3D<> temp;
	rw::math::Vector3D<> init_current_belt_;
	rw::math::Vector3D<> current_belt_;
	rw::math::Vector3D<> desired_belt_;
	rw::math::Vector3D<> error_;

	rw::math::Transform3D<> tf_bearing_current_belt_;
	rw::math::Transform3D<> tf_bearing_desired_belt_;

	rw::math::Transform3D<> tf_tcp_current_belt_;
	rw::math::Transform3D<> tf_tcp_desired_belt_;



};



#endif /* SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_TASK_ROBOT_H_ */
