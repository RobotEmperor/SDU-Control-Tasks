/*
 * ur_robot.cpp
 *
 *  Created on: Jul 16, 2020
 *      Author: yik
 */

#include "ur_robot.h"

UrRobot::UrRobot(double init_control_time):
control_time_(init_control_time)
{
  WorkCell::Ptr wc;
  SerialDevice::Ptr device;

  wc = WorkCellLoader::Factory::load(WC_FILE);
  device = wc->findDevice<SerialDevice>("UR10e");

  if (wc.isNull())
    RW_THROW("WorkCell could not be loaded.");
  if (device.isNull())
    RW_THROW("UR10e device could not be found.");

  state_  = wc->getDefaultState();
  solver_ = std::make_shared<ClosedFormIKSolverUR>(device, state_);
  solver_->setCheckJointLimits(true);

  time_count_ = 0;
  motion_time_ = 5;
  f_kp_ = 0;
  f_ki_ = 0;
  f_kd_ = 0;

  p_kp_ = 0;
  p_ki_ = 0;
  p_kd_ = 0;

  contact_check_ = false;

  //solution check
  joint_vel_limits_ = false;

  //tool_estimation
  tool_estimation = std::make_shared<ToolEstimation>();
  tool_estimation->initialize();

  //statistics
  statistics_math = std::make_shared<StatisticsMath>();

  // fifth order traj
  ur10e_traj = std::make_shared<EndEffectorTraj>();
  ur10e_task = std::make_shared<TaskMotion>();

  //control
  force_x_compensator = std::make_shared<PID_function>(control_time_, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);
  force_y_compensator = std::make_shared<PID_function>(control_time_, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);
  force_z_compensator = std::make_shared<PID_function>(control_time_, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);

  position_x_controller = std::make_shared<PID_function>(control_time_, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);
  position_y_controller = std::make_shared<PID_function>(control_time_, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);
  position_z_controller = std::make_shared<PID_function>(control_time_, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);

  previous_task_command_ = "";

  actual_joint_positions_.resize(6,0);
  actual_tcp_pose_.resize(6,0);
  target_tcp_pose_.resize(6,0);
  acutal_tcp_acc_.resize(3,0);
  raw_ft_data_.resize(6,0);
  filtered_tcp_ft_data_.resize(6,0);
  contacted_ft_data_.resize(6,0);
  contacted_ft_no_offset_data_.resize(6,0);
  current_q_.resize(6,0);
  pid_compensation_.resize(6,0);

  //control states
  set_point_vector_.resize(6,0);
  desired_pose_vector_.resize(6,0);
  desired_force_torque_vector_.resize(6,0);
  compensated_pose_vector_.resize(6,0);

}
UrRobot::~UrRobot()
{

}
void UrRobot::load_initialize_parameter(std::string path_)
{
  // have to fix for automatic calculation
  //robot A
  current_q_[0] = 2.6559;
  current_q_[1] = -1.85396;
  current_q_[2] = -2.03763;
  current_q_[3] = -2.38718;
  current_q_[4] = -2.84058;
  current_q_[5] = -3.13698;

  solution_number_ = 1;

  mass_ = 1.75;

  ur10e_task->initialize(control_time_, path_);

  //robot A
  desired_pose_vector_ = ur10e_task -> get_current_pose();

  //setting up control time and mass of tool
  tool_estimation ->set_parameters(control_time_, mass_);
  ur10e_traj->set_control_time(control_time_);
}
void UrRobot::load_task_motion(std::string path_, std::string task_)
{
  ur10e_task->load_task_motion(path_, task_);
}
void UrRobot::load_tcp_task_motion(std::string path_)
{
  ur10e_task->trans_tcp_to_base_motion(path_);
}
void UrRobot::simulation_initialize()
{
  gazebo_check_ = true;

  std::cout << COLOR_GREEN_BOLD << "Gazebo robot start " << COLOR_RESET << std::endl;

  //robot A
  compensated_pose_vector_[0] = -0.6649675947900577;
  compensated_pose_vector_[1] = 0.2054059150102794;
  compensated_pose_vector_[2] = 0.25997552421325143;

  compensated_pose_vector_[3] = -0.7298511759689527;
  compensated_pose_vector_[4] = -1.7577922996928013;
  compensated_pose_vector_[5] = 1.760382318187891;
}

void UrRobot::initialize(std::string robot_ip)
{
  rtde_receive = std::make_shared<RTDEReceiveInterface>(robot_ip);
  rtde_control = std::make_shared<RTDEControlInterface>(robot_ip);
  rtde_control->zeroFtSensor();

  usleep(2000000);

  std::cout << COLOR_YELLOW_BOLD << "Robot A connected to your program" << COLOR_RESET << std::endl;
  std::cout << COLOR_RED_BOLD << "Robot A will move 2 seconds later" << COLOR_RESET << std::endl;

  rtde_control->moveJ(current_q_,0.1,0.1); // move to initial pose
  std::cout << COLOR_RED_BOLD << "Send" << COLOR_RESET << std::endl;
  usleep(3000000);
}
void UrRobot::init_accelerometer()
{
  std::cout << COLOR_GREEN_BOLD << "Real robot start, will compensate acc sensor's gravity " << COLOR_RESET << std::endl;
  //getting sensor values sensor filter
  acutal_tcp_acc_  = rtde_receive->getActualToolAccelerometer();
  actual_tcp_pose_ = rtde_receive->getActualTCPPose();

  tf_current_ = Transform3D<> (Vector3D<>(actual_tcp_pose_[0], actual_tcp_pose_[1], actual_tcp_pose_[2]), EAA<>(actual_tcp_pose_[3], actual_tcp_pose_[4], actual_tcp_pose_[5]).toRotation3D());

  tool_estimation->set_orientation_data(tf_current_);
  tool_estimation->set_gravity_input_data(acutal_tcp_acc_);

  std::cout << acutal_tcp_acc_ << std::endl;
}
void UrRobot::controller()
{

  time_count_ += control_time_;

  //ros_state->update_ros_data();

  //do something
  //check if task command is received or not
  //run motion trajectory task_motion
  //auto_task_motion goal
  if(!current_task_command_.compare("auto"))
  {
    static int previous_phases_ = 0;
    ur10e_task->auto_task_motion(contact_check_);

    if(ur10e_task->get_phases_() == 1 && previous_phases_ == 2)
    {
      cout << "Zero ft sensor" << endl;
      tool_estimation->set_sensor_offset_value(raw_ft_data_);
    }
    previous_phases_ = ur10e_task->get_phases_();
  }
  else
  {
    if(!current_task_command_.compare("set_point"))
    {
      //      static double set_point_motion_time_ = 0.0;
      //      set_point_motion_time_ = ros_state->get_set_point()[6];
      //      ur10e_task->tf_set_point_base(ros_state->get_set_point());
      //      set_point_vector = ur10e_task->get_set_point_base();
      //      ur10e_task->set_point(set_point_vector_[0], set_point_vector_[1], set_point_vector_[2], set_point_vector_[3], set_point_vector_[4], set_point_vector_[5], set_point_motion_time_);
      //      ur10e_task->generate_trajectory();
      //      ros_state->clear_task_command();
    }
    else
    {
      if(current_task_command_.compare(previous_task_command_) != 0 && current_task_command_.compare("") != 0)
      {
        ur10e_task->clear_phase();
        ur10e_task->clear_task_motion();
        ur10e_task->change_motion(current_task_command_);
        contact_check_ = false;
      }
      ur10e_task->run_task_motion();
      ur10e_task->generate_trajectory();
    }
  }
  //motion reference
  desired_pose_vector_ = ur10e_task->get_current_pose();
  desired_force_torque_vector_ = ur10e_task->get_desired_force_torque();

  //controller for pose controller
  position_x_controller->set_pid_gain(p_kp_,p_ki_,p_kd_);
  position_y_controller->set_pid_gain(p_kp_,p_ki_,p_kd_);
  position_z_controller->set_pid_gain(p_kp_,p_ki_,p_kd_);

  //controller for force compensation
  force_x_compensator->set_pid_gain(f_kp_,f_ki_,f_kd_);
  force_y_compensator->set_pid_gain(f_kp_,f_ki_,f_kd_);
  force_z_compensator->set_pid_gain(f_kp_,f_ki_,f_kd_);

  if(!gazebo_check_)
  {
    //getting sensor values sensor filter
    raw_ft_data_     = rtde_receive->getActualTCPForce();
    acutal_tcp_acc_  = rtde_receive->getActualToolAccelerometer();
    actual_tcp_pose_ = rtde_receive->getActualTCPPose();
    actual_joint_positions_  = rtde_receive->getActualQ();

    ur10e_task->set_current_pose_eaa(actual_tcp_pose_[0], actual_tcp_pose_[1], actual_tcp_pose_[2],actual_tcp_pose_[3], actual_tcp_pose_[4], actual_tcp_pose_[5]);

    tf_current_ = Transform3D<> (Vector3D<>(actual_tcp_pose_[0], actual_tcp_pose_[1], actual_tcp_pose_[2]), EAA<>(actual_tcp_pose_[3], actual_tcp_pose_[4], actual_tcp_pose_[5]).toRotation3D());

    //force torque sensor filtering
    tool_estimation->set_orientation_data(tf_current_);
    tool_estimation->process_estimated_force(raw_ft_data_, acutal_tcp_acc_);

    contacted_ft_data_ = tool_estimation->get_contacted_force();
    current_ft_ = Wrench6D<> (contacted_ft_data_[0], contacted_ft_data_[1], contacted_ft_data_[2], contacted_ft_data_[3], contacted_ft_data_[4], contacted_ft_data_[5]);
    current_ft_ = (tf_current_.R()).inverse()*current_ft_;
    tf_current_ = Transform3D<> (Vector3D<>(actual_tcp_pose_[0], actual_tcp_pose_[1], actual_tcp_pose_[2]), EAA<>(actual_tcp_pose_[3], actual_tcp_pose_[4], actual_tcp_pose_[5]).toRotation3D());


    if(current_ft_.force()[1] > 5  && !contact_check_)
    {
      contact_check_ = 1;
      cout << "A rubber belt was inserted in a pulley" << endl;
    }

    position_x_controller->PID_calculate(desired_pose_vector_[0], actual_tcp_pose_[0], 0);
    position_y_controller->PID_calculate(desired_pose_vector_[1], actual_tcp_pose_[1], 0);
    position_z_controller->PID_calculate(desired_pose_vector_[2], actual_tcp_pose_[2], 0);

    force_x_compensator->PID_calculate(ur10e_task->get_desired_force_torque()[0],current_ft_.force()[0],0);
    force_y_compensator->PID_calculate(ur10e_task->get_desired_force_torque()[1],current_ft_.force()[1],0);
    force_z_compensator->PID_calculate(ur10e_task->get_desired_force_torque()[2],current_ft_.force()[2],0);

    tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(force_x_compensator->get_final_output(),force_y_compensator->get_final_output(),force_z_compensator->get_final_output()), EAA<>(0,0,0).toRotation3D());

    tf_modified_pose_ = tf_current_ * tf_tcp_desired_pose_;

    tf_modified_pose_.P() = tf_current_.P() - tf_modified_pose_.P();

    pid_compensation_[0] = position_x_controller->get_final_output();
    pid_compensation_[1] = position_y_controller->get_final_output();
    pid_compensation_[2] = position_z_controller->get_final_output();
    pid_compensation_[3] = (tf_modified_pose_.P())[0];
    pid_compensation_[4] = (tf_modified_pose_.P())[1];
    pid_compensation_[5] = (tf_modified_pose_.P())[2];


    filtered_tcp_ft_data_[0] = current_ft_.force()[0];
    filtered_tcp_ft_data_[1] = current_ft_.force()[1];
    filtered_tcp_ft_data_[2] = current_ft_.force()[2];
    filtered_tcp_ft_data_[3] = current_ft_.torque()[0];
    filtered_tcp_ft_data_[4] = current_ft_.torque()[1];
    filtered_tcp_ft_data_[5] = current_ft_.torque()[2];

    compensated_pose_vector_[0] = actual_tcp_pose_[0] + position_x_controller->get_final_output()+(tf_modified_pose_.P())[0];
    compensated_pose_vector_[1] = actual_tcp_pose_[1] + position_y_controller->get_final_output()+(tf_modified_pose_.P())[1];
    compensated_pose_vector_[2] = actual_tcp_pose_[2] + position_z_controller->get_final_output()+(tf_modified_pose_.P())[2];

    compensated_pose_vector_[3] = desired_pose_vector_[3]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector_[4] = desired_pose_vector_[4]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector_[5] = desired_pose_vector_[5]; //+ force_x_compensator->get_final_output();
  }
  else
  {
    position_x_controller->PID_calculate(desired_pose_vector_[0], compensated_pose_vector_[0], 0);
    position_y_controller->PID_calculate(desired_pose_vector_[1], compensated_pose_vector_[1], 0);
    position_z_controller->PID_calculate(desired_pose_vector_[2], compensated_pose_vector_[2], 0);

    //contact_check_ = ros_state->get_test();
    ur10e_task->set_current_pose_eaa(compensated_pose_vector_[0], compensated_pose_vector_[1], compensated_pose_vector_[2],compensated_pose_vector_[3], compensated_pose_vector_[4], compensated_pose_vector_[5]);

    pid_compensation_[0] = position_x_controller->get_final_output();
    pid_compensation_[1] = position_y_controller->get_final_output();
    pid_compensation_[2] = position_z_controller->get_final_output();
    pid_compensation_[3] = 0;
    pid_compensation_[4] = 0;
    pid_compensation_[5] = 0;

    //cout << pid_compensation << endl;

    compensated_pose_vector_[0] = compensated_pose_vector_[0] + pid_compensation_[0];
    compensated_pose_vector_[1] = compensated_pose_vector_[1] + pid_compensation_[1];
    compensated_pose_vector_[2] = compensated_pose_vector_[2] + pid_compensation_[2];

    compensated_pose_vector_[3] = desired_pose_vector_[3]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector_[4] = desired_pose_vector_[4]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector_[5] = desired_pose_vector_[5]; //+ force_x_compensator->get_final_output();
  }


  tf_desired_ = Transform3D<> (Vector3D<>(compensated_pose_vector_[0], compensated_pose_vector_[1], compensated_pose_vector_[2]),
      EAA<>(compensated_pose_vector_[3], compensated_pose_vector_[4], compensated_pose_vector_[5]).toRotation3D());

  //solve ik problem

  solutions_ = solver_->solve(tf_desired_, state_);

  //from covid - robot
  for (unsigned int num = 0; num < 6; num ++)
  {
    // LOG_DEBUG("[%s]", "Wrapping for joint");
    const double diffOrig = fabs(current_q_[num] - solutions_[solution_number_][num]);
    // LOG_DEBUG("[diffOrig: %e , %e]", (solutions[i][j])/rw::math::Pi*180.0, diffOrig );

    const double diffAdd = fabs(current_q_[num] - (solutions_[solution_number_][num] + 2 * rw::math::Pi));
    // LOG_DEBUG("[diffAdd: %e , %e]", (solutions[i][j]+2*rw::math::Pi)/rw::math::Pi*180.0, diffAdd );

    const double diffSub = fabs(current_q_[num] - (solutions_[solution_number_][num] - 2 * rw::math::Pi));
    // LOG_DEBUG("[diffSub: %e , %e]", (solutions[i][j]-2*rw::math::Pi)/rw::math::Pi*180.0, diffSub );

    if (diffAdd < diffOrig && diffAdd < diffSub)
    {
      solutions_[solution_number_][num] += 2 * rw::math::Pi;
    }
    else if (diffSub < diffOrig && diffSub < diffAdd)
    {
      solutions_[solution_number_][num] -= 2 * rw::math::Pi;
    }
  }

  //check velocity
  for(int num = 0; num <6 ; num ++)
  {
    if(fabs((solutions_[solution_number_].toStdVector()[num] - current_q_[num])/control_time_) > 270*DEGREE2RADIAN)
    {
      std::cout << "::" << num << "::" << fabs((solutions_[solution_number_].toStdVector()[num] - current_q_[num])/control_time_) << std::endl;
      std::cout << COLOR_RED_BOLD << "Robot speed is so FAST" << COLOR_RESET << std::endl;
      joint_vel_limits_ = true;
    }
  }

  //send command in joint space to ur robot or gazebo
  if(!joint_vel_limits_)
  {
    if(!gazebo_check_)
    {
      rtde_control->servoJ(solutions_[solution_number_].toStdVector(),0,0,control_time_,0.04,2000);
    }
    for(int num = 0; num <6 ; num ++)
    {
      current_q_[num] = solutions_[solution_number_].toStdVector()[num];
    }
  }

  previous_task_command_ = current_task_command_;
}
void UrRobot::robot_servo_stop()
{
  rtde_control->servoStop();
}
void UrRobot::set_current_task_motion(std::string command_)
{
  current_task_command_ = command_;
}
void UrRobot::set_position_gain(double p_, double i_, double d_)
{
  p_kp_ = p_;
  p_ki_ = i_;
  p_kd_ = d_;
}
void UrRobot::set_force_gain(double p_, double i_, double d_)
{
  f_kp_ = p_;
  f_ki_ = i_;
  f_kd_ = d_;
}
void UrRobot::set_test_contact(bool check_)
{
  contact_check_ = check_;
}
double UrRobot::get_time_count()
{
  return time_count_;
}
std::vector<double> UrRobot::get_raw_ft_data()
{
  return raw_ft_data_;
}
std::vector<double> UrRobot::get_actual_joint_positions()
{
  return actual_joint_positions_;
}
std::vector<double> UrRobot::get_actual_tcp_pose()
{
  return actual_tcp_pose_;
}
std::vector<double> UrRobot::get_target_tcp_pose()
{
  return target_tcp_pose_;
}
std::vector<double> UrRobot::get_acutal_tcp_acc()
{
  return acutal_tcp_acc_;
}
std::vector<double> UrRobot::get_pid_compensation()
{
  return pid_compensation_;
}
std::vector<double> UrRobot::get_desired_pose_vector()
{
  return desired_pose_vector_;
}
std::vector<double> UrRobot::get_filtered_tcp_ft_data()
{
  return filtered_tcp_ft_data_;
}
std::vector<double> UrRobot::get_contacted_ft_data()
{
  return contacted_ft_data_;
}
std::vector<double> UrRobot::get_desired_q_()
{
  return current_q_;
}
