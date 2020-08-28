/*
 * task_robot.cpp
 *
 *  Created on: Aug 19, 2020
 *      Author: yik
 */

#include "task_robot.h"
TaskRobot::TaskRobot()
{

}
TaskRobot::TaskRobot(std::string robot_name, std::string init_path)
{
  control_time_ = 0.002;

  //vectors
  joint_positions_.assign(6,0); //(6);
  actual_tcp_pose_.assign(6,0); //(6);
  actual_tcp_speed_.assign(6,0); //(6);
  target_tcp_pose_.assign(6,0); //(6);
  acutal_tcp_acc_.assign(3,0); //(3);
  raw_ft_data_.assign(6,0); //(6);
  filtered_tcp_ft_data_.assign(6,0); //(6);
  contacted_ft_data_.assign(6,0); //(6);
  contacted_ft_no_offset_data_.assign(6,0); //(6);
  current_q_.assign(6,0); //(6);
  pid_compensation_.assign(6,0); //(6);

  set_point_vector_.assign(6,0); //(6);
  desired_pose_vector_.assign(6,0); //(6);
  desired_force_torque_vector_.assign(6,0); //(6);
  compensated_pose_vector_.assign(6,0); //(6);
  error_ee_pose_.assign(6,0); //(6);
  compensated_q_.assign(6,0); //(6);

  data_current_belt_.assign(3,0);
  data_desired_belt_.assign(3,0);

  force_controller_gain_.x_kp = 0;
  force_controller_gain_.x_ki = 0;
  force_controller_gain_.x_kd = 0;
  force_controller_gain_.y_kp = 0;
  force_controller_gain_.y_ki = 0;
  force_controller_gain_.y_kd = 0;
  force_controller_gain_.z_kp = 0;
  force_controller_gain_.z_ki = 0;
  force_controller_gain_.z_kd = 0;
  force_controller_gain_.eaa_x_kp = 0;
  force_controller_gain_.eaa_x_ki = 0;
  force_controller_gain_.eaa_x_kd = 0;
  force_controller_gain_.eaa_y_kp = 0;
  force_controller_gain_.eaa_y_ki = 0;
  force_controller_gain_.eaa_y_kd = 0;
  force_controller_gain_.eaa_z_kp = 0;
  force_controller_gain_.eaa_z_ki = 0;
  force_controller_gain_.eaa_z_kd = 0;

  position_controller_gain_.x_kp = 0;
  position_controller_gain_.x_ki = 0;
  position_controller_gain_.x_kd = 0;
  position_controller_gain_.y_kp = 0;
  position_controller_gain_.y_ki = 0;
  position_controller_gain_.y_kd = 0;
  position_controller_gain_.z_kp = 0;
  position_controller_gain_.z_ki = 0;
  position_controller_gain_.z_kd = 0;
  position_controller_gain_.eaa_x_kp = 0;
  position_controller_gain_.eaa_x_ki = 0;
  position_controller_gain_.eaa_x_kd = 0;
  position_controller_gain_.eaa_y_kp = 0;
  position_controller_gain_.eaa_y_ki = 0;
  position_controller_gain_.eaa_y_kd = 0;
  position_controller_gain_.eaa_z_kp = 0;
  position_controller_gain_.eaa_z_ki = 0;
  position_controller_gain_.eaa_z_kd = 0;

  robot_name_ = robot_name;

  //data log
  data_log_ = std::make_shared<DataLogging>(init_path + "/" + robot_name_);
  data_log_->initialize();

  //tool_estimation
  tool_estimation_  = std::make_shared<ToolEstimation>();
  tool_estimation_->initialize();

  //statistics
  statistics_math_ = std::make_shared<StatisticsMath>();

  // fifth order traj
  robot_fifth_traj_ = std::make_shared<EndEffectorTraj>();
  robot_task_ = std::make_shared<TaskMotion>();

  //setting up control time and mass of tool
  //tool_estimation_ ->set_parameters(control_time_, tool_mass_);
  robot_fifth_traj_->set_control_time(control_time_);

  robot_task_->initialize(control_time_, init_path + "/" + robot_name_ + "/initialize_robot.yaml");

  //load motion data
  robot_task_->load_task_motion(init_path + "/" + robot_name_ + "/motion/initialize.yaml", "initialize");
  robot_task_->trans_tcp_to_base_motion(init_path + "/" + robot_name_ + "/motion/tcp_belt_task.yaml");

  desired_pose_vector_ = robot_task_ -> get_current_pose();

  //control
  force_x_compensator_ = std::make_shared<PID_function>(control_time_, 0.0075, -0.0075, 0, 0, 0, 0.0000001, -0.0000001);
  force_y_compensator_ = std::make_shared<PID_function>(control_time_, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);
  force_z_compensator_ = std::make_shared<PID_function>(control_time_, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);

  position_x_controller_ = std::make_shared<PID_function>(control_time_, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);
  position_y_controller_ = std::make_shared<PID_function>(control_time_, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);
  position_z_controller_ = std::make_shared<PID_function>(control_time_, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);

  control_check_ = false;
  joint_vel_limits_ = false;
  contact_check_ = false;

  preferred_solution_number_ = 0;
  time_count_ = 0;
  previous_task_command_ = "";

  gazebo_check_ = true;
  belt_robust_value_ = 0;
  change_x_ = 0;
  change_y_ = 0;
  change_z_ = 0;
  current_belt_x_= 0;
  current_belt_y_= 0;
  current_belt_z_= 0;
  desired_belt_x_ = 0;
  desired_belt_y_ = 0;
  desired_belt_z_ = 0;

  flag = false;

  tf_moveable_robot_to_init_belt_ = Transform3D<> (Vector3D<>(0,0,0), EAA<>(0,0,0).toRotation3D());
  tf_bearing_to_rubber_point = Transform3D<> (Vector3D<>(0,0,0), EAA<>(0,0,0).toRotation3D());
}
TaskRobot::~TaskRobot()
{

}
void TaskRobot::init_model(std::string wc_file, std::string robot_model)
{
  wc_ = WorkCellLoader::Factory::load(wc_file);
  device_ = wc_->findDevice<SerialDevice>(robot_model);

  if (wc_.isNull())
    RW_THROW("WorkCell could not be loaded.");
  if (device_.isNull())
    RW_THROW("UR10e device could not be found.");

  state_ = wc_->getDefaultState();
  solver_ = std::make_shared<ClosedFormIKSolverUR>(device_, state_);
  solver_->setCheckJointLimits(true);
}
void TaskRobot::initialize(std::string robot_ip, bool gazebo_check)
{
  gazebo_check_ = gazebo_check;
  if(!gazebo_check_)
  {
    //set up the robot
    rtde_receive_ = std::make_shared<RTDEReceiveInterface>(robot_ip);
    rtde_control_ = std::make_shared<RTDEControlInterface>(robot_ip);

    usleep(1000000);

    if(rtde_control_->zeroFtSensor())
    {
      usleep(1000000);
      std::cout << COLOR_GREEN_BOLD << robot_name_ << ": Completed " << "ZeroFtSensor" << COLOR_RESET << std::endl;
    }
    else
      std::cout << COLOR_RED_BOLD << robot_name_ <<": Error " << "ZeroFtSensor" << COLOR_RESET << std::endl;

    std::cout << COLOR_GREEN_BOLD << robot_name_ <<": Connected to your program" << COLOR_RESET << std::endl;
  }
}

void TaskRobot::move_to_init_pose()
{
  if(!gazebo_check_)
  {
    std::cout << COLOR_RED_BOLD << robot_name_ <<": will move 1 seconds later" << COLOR_RESET << std::endl;

    usleep(1000000);

    rtde_control_->moveL(desired_pose_vector_, 0.05, 0.05);

    std::cout << COLOR_RED_BOLD << robot_name_ <<": Send" << COLOR_RESET << std::endl;

    usleep(3000000);

    std::cout << COLOR_GREEN_BOLD << robot_name_ << ": Adjust Accelerometer Sensor and compensate gravity term" << COLOR_RESET << std::endl;
    //getting sensor values sensor filter
    acutal_tcp_acc_  = rtde_receive_->getActualToolAccelerometer();
    actual_tcp_pose_ = rtde_receive_->getActualTCPPose();

    tf_current_ = Transform3D<> (Vector3D<>(actual_tcp_pose_[0], actual_tcp_pose_[1], actual_tcp_pose_[2]), EAA<>(actual_tcp_pose_[3], actual_tcp_pose_[4], actual_tcp_pose_[5]).toRotation3D());

    tool_estimation_->set_orientation_data(tf_current_);
    tool_estimation_->set_gravity_input_data(acutal_tcp_acc_);

    std::cout << robot_name_ << ": Compensated gravity terms " << acutal_tcp_acc_ << std::endl;
    std::cout << COLOR_GREEN << robot_name_ << " All of things were initialized!" << COLOR_RESET << std::endl;
  }
  else
  {
    compensated_pose_vector_ = desired_pose_vector_;
    tf_current_ = Transform3D<> (Vector3D<>(compensated_pose_vector_[0], compensated_pose_vector_[1], compensated_pose_vector_[2]), EAA<>(compensated_pose_vector_[3], compensated_pose_vector_[4], compensated_pose_vector_[5]).toRotation3D());
  }
}
void TaskRobot::estimation_of_belt_position(double radious)
{

  if(!flag && robot_task_->get_phases_() == 2)
  {


    temp = tf_base_to_bearing_;
    temp.invMult(temp, tf_current_);
    tf_bearing_to_moveable_robot_start = temp;



    //bearing robot point
    init_current_belt_[0] = 0;
    init_current_belt_[1] = tf_bearing_to_moveable_robot_start.P()[1] + 0.008; // same in two robot
    init_current_belt_[2] = tf_bearing_to_moveable_robot_start.P()[2];

    desired_belt_[1] = radious; // radious 0.031 gripper 66 %
    desired_belt_[2] = -0.004;

  if(!robot_name_.compare("robot_B"))
    {
    desired_belt_[0] = -0.01;
    }
    else
    {
    desired_belt_[0] = 0;
    }


    data_desired_belt_[0] = desired_belt_[0];
    data_desired_belt_[1] = desired_belt_[1];
    data_desired_belt_[2] = desired_belt_[2];

    cout << " init_current_belt_  :"<< init_current_belt_ << endl;

    tf_bearing_current_belt_.P() = init_current_belt_;
    tf_bearing_desired_belt_.P() = desired_belt_;

    temp = tf_bearing_to_moveable_robot_start;
    temp.invMult(temp, tf_bearing_current_belt_);
    tf_moveable_robot_to_init_belt_ = temp;

    flag = true;
  }

  temp = tf_base_to_bearing_static_robot_;
  temp.invMult(temp, tf_base_to_static_robot_);
  tf_bearing_to_static_robot = temp;

  temp = tf_base_to_bearing_;
  temp.invMult(temp, tf_current_);
  tf_bearing_to_moveable_robot = temp;

  temp = tf_bearing_to_moveable_robot;
  temp.invMult(temp, tf_bearing_current_belt_);
  tf_tcp_current_belt_.P() = temp.P();

  temp = tf_bearing_to_moveable_robot;
  temp.invMult(temp, tf_bearing_desired_belt_);
  tf_tcp_desired_belt_.P()  = temp.P();


  error_ = tf_tcp_desired_belt_.P() - tf_tcp_current_belt_.P();

  cout << " error_  :"<< error_<< endl;

  change_x_ = error_[0];
  change_y_ = error_[1];
  change_z_ = error_[2];

  cout << " change_x_  :"<< change_x_ << endl;
  cout << " change_y_  :"<< change_y_ << endl;
  cout << " change_z_  :"<< change_z_ << endl;

}
bool TaskRobot::tasks(std::string command)
{
  static rw::math::Vector3D<> tcp_non_rotated_desired;

  static bool task_check = false;

  if(!command.compare(""))
    return false;

  if(!command.compare("auto"))
  {
    task_check = robot_task_->make_belt_robust(belt_robust_value_);

    if(!robot_name_.compare("robot_A"))
    {
      task_check = robot_task_-> close_to_pulleys(0,0.01,0.027);
      if(robot_task_->get_phases_() == 2)
      {
        if(!flag)
          estimation_of_belt_position(0.0195);
        task_check = robot_task_->insert_belt_into_pulley(contact_check_, change_x_, change_y_, change_z_);
        desired_force_torque_vector_[0] = 10;
        desired_force_torque_vector_[2] = -12;

        temp = tf_base_to_bearing_;
        temp.invMult(temp, tf_current_);
        tf_bearing_to_moveable_robot = temp;

        tf_bearing_to_rubber_point = tf_bearing_to_moveable_robot*tf_moveable_robot_to_init_belt_;
        current_belt_ = tf_bearing_to_rubber_point.P();
        data_current_belt_[0] = current_belt_[0];
        data_current_belt_[1] = current_belt_[1];
        data_current_belt_[2] = current_belt_[2];

      }
      if(robot_task_->get_phases_() == 3)
      {
        task_check = robot_task_->up_motion(contact_check_, 0, -0.03, 0.011, 0, 0, 0);
        desired_force_torque_vector_[0] = 10;
        desired_force_torque_vector_[2] = -12;
      }
      if(robot_task_->get_phases_() == 4)
      {
        robot_task_->finish_1(contact_check_, 0.02, 0, 0, 0, 0, 0);
        //task_check = robot_task_->up_motion(contact_check_, 0.005, 0, 0, 0, 0, 0);
      }
      if(robot_task_->get_phases_() == 5)
      {
        robot_task_->finish_2(contact_check_, 0, 0, -0.04, 0, 0, 0);
        //task_check = robot_task_->up_motion(contact_check_, 0.005, 0, 0, 0, 0, 0);
      }
      //std::cout << robot_task_->get_phases_()  << std::endl;
    }

    if(!robot_name_.compare("robot_B"))
    {

      //cout << robot_task_->get_phases_() << endl;

      if(previous_phase_ != robot_task_->get_phases_() && robot_task_->get_phases_() == 1)
      {
        rw::math::Transform3D<> tf_tcp_rotated_;

        tcp_non_rotated_desired[0] = 0;
        tcp_non_rotated_desired[1] = 0;
        tcp_non_rotated_desired[2] = 0;

        tf_tcp_rotated_ = Transform3D<> (Vector3D<>(0, 0, 0), RPY<>(0, 25*DEGREE2RADIAN,0).toRotation3D());

        tcp_non_rotated_desired  = tf_tcp_rotated_ * tcp_non_rotated_desired;
        task_check = robot_task_-> close_to_pulleys(tcp_non_rotated_desired[0],tcp_non_rotated_desired[1],tcp_non_rotated_desired[2]);
      }
      if(previous_phase_ != robot_task_->get_phases_() && robot_task_->get_phases_() == 3)
      {
        rw::math::Transform3D<> tf_tcp_rotated_;

        tcp_non_rotated_desired[0] = -0.005;
        tcp_non_rotated_desired[1] = -0.064;
        tcp_non_rotated_desired[2] = -0.005;

        tf_tcp_rotated_ = Transform3D<> (Vector3D<>(0, 0, 0), RPY<>(0, 25*DEGREE2RADIAN,0).toRotation3D());

        tcp_non_rotated_desired  = tf_tcp_rotated_ * tcp_non_rotated_desired;

        //task_check = robot_task_->up_motion(contact_check_, tcp_non_rotated_desired[0], tcp_non_rotated_desired[1], tcp_non_rotated_desired[2], 0, 0, 0);
        //desired_force_torque_vector_[0] = -16;
        //desired_force_torque_vector_[1] = 24;
      }

      if(robot_task_->get_phases_() == 2)
      {
        if(!flag)
          estimation_of_belt_position(0.0319);
        task_check = robot_task_->insert_belt_into_pulley(contact_check_, change_x_, change_y_, change_z_);

        desired_force_torque_vector_[0] = 0;

        temp = tf_base_to_bearing_;
        temp.invMult(temp, tf_current_);
        tf_bearing_to_moveable_robot = temp;

        tf_bearing_to_rubber_point = tf_bearing_to_moveable_robot*tf_moveable_robot_to_init_belt_;
        current_belt_ = tf_bearing_to_rubber_point.P();
        data_current_belt_[0] = current_belt_[0];
        data_current_belt_[1] = current_belt_[1];
        data_current_belt_[2] = current_belt_[2];
      }
      if(robot_task_->get_phases_() == 3)
      {
        task_check = robot_task_->up_motion(contact_check_, tcp_non_rotated_desired[0], tcp_non_rotated_desired[1], tcp_non_rotated_desired[2], 0, 5*DEGREE2RADIAN, 0);
        desired_force_torque_vector_[0] = 0;
        desired_force_torque_vector_[1] = 24;
        //std::cout << current_ft_.force()<< std::endl;
      }
      if(robot_task_->get_phases_() == 4)
      {
        desired_force_torque_vector_[1] = 0;
        robot_task_->finish_1(contact_check_,  -0.005,0,0, 0, 0 ,0);
        //task_check = robot_task_->up_motion(contact_check_, 0.005, 0, 0, 0, 0, 0);
      }
      if(robot_task_->get_phases_() == 5)
      {
        robot_task_->finish_2(contact_check_, 0, 0, -0.04, 0, 0, 0);
        //task_check = robot_task_->up_motion(contact_check_, 0.005, 0, 0, 0, 0, 0);
      }

    }

    previous_phase_ = robot_task_->get_phases_();
    robot_task_->generate_trajectory();
    robot_task_->check_phases();
  }
  else
  {
    if(command.compare(previous_task_command_) != 0 && command.compare("") != 0)
    {
      robot_task_->clear_phase();
      robot_task_->clear_task_motion();
      robot_task_->change_motion(command);
      contact_check_ = false;
    }
    else
    {
      robot_task_->run_task_motion();
    }
    robot_task_->generate_trajectory();
  }

  previous_task_command_ = command;

  return task_check;
}
bool TaskRobot::hybrid_controller()
{
  //motion reference

  desired_pose_vector_ = robot_task_->get_current_pose();

  //desired_force_torque_vector_ = robot_task_->get_desired_force_torque();

  //controller for pose controller
  position_x_controller_->set_pid_gain(position_controller_gain_.x_kp,position_controller_gain_.x_ki,position_controller_gain_.x_kd);
  position_y_controller_->set_pid_gain(position_controller_gain_.x_kp,position_controller_gain_.x_ki,position_controller_gain_.x_kd);
  position_z_controller_->set_pid_gain(position_controller_gain_.x_kp,position_controller_gain_.x_ki,position_controller_gain_.x_kd);

  //controller for force compensation
  force_x_compensator_->set_pid_gain(force_controller_gain_.x_kp,force_controller_gain_.x_ki,force_controller_gain_.x_kd);
  force_y_compensator_->set_pid_gain(force_controller_gain_.x_kp,force_controller_gain_.x_ki,force_controller_gain_.x_kd);
  force_z_compensator_->set_pid_gain(force_controller_gain_.x_kp,force_controller_gain_.x_ki,force_controller_gain_.x_kd);

  if(!gazebo_check_)
  {
    //getting sensor values sensor filter
    raw_ft_data_     = rtde_receive_->getActualTCPForce();
    acutal_tcp_acc_  = rtde_receive_->getActualToolAccelerometer();
    actual_tcp_pose_ = rtde_receive_->getActualTCPPose();
    actual_tcp_speed_ = rtde_receive_->getActualTCPSpeed();
    joint_positions_ = rtde_receive_->getActualQ();
    //current_q_ = rtde_receive_->getActualQ();

    robot_task_->set_current_pose_eaa(actual_tcp_pose_[0], actual_tcp_pose_[1], actual_tcp_pose_[2],actual_tcp_pose_[3], actual_tcp_pose_[4], actual_tcp_pose_[5]);

    tf_current_ = Transform3D<> (Vector3D<>(actual_tcp_pose_[0], actual_tcp_pose_[1], actual_tcp_pose_[2]), EAA<>(actual_tcp_pose_[3], actual_tcp_pose_[4], actual_tcp_pose_[5]).toRotation3D());

    //force torque sensor filtering
    tool_estimation_->set_orientation_data(tf_current_);
    tool_estimation_->process_estimated_force(raw_ft_data_, acutal_tcp_acc_);

    contacted_ft_data_ = tool_estimation_->get_contacted_force();
    current_ft_ = Wrench6D<> (contacted_ft_data_[0], contacted_ft_data_[1], contacted_ft_data_[2], contacted_ft_data_[3], contacted_ft_data_[4], contacted_ft_data_[5]);
    current_ft_ = (tf_current_.R()).inverse()*current_ft_;
    tf_current_ = Transform3D<> (Vector3D<>(actual_tcp_pose_[0], actual_tcp_pose_[1], actual_tcp_pose_[2]), EAA<>(actual_tcp_pose_[3], actual_tcp_pose_[4], actual_tcp_pose_[5]).toRotation3D());

    if(current_ft_.force()[1] > 5  && !contact_check_) // tool frame
    {
      contact_check_ = 1;
      std::cout << "A rubber belt was inserted in a pulley" << std::endl;
    }

    position_x_controller_->PID_calculate(desired_pose_vector_[0], actual_tcp_pose_[0], 0);
    position_y_controller_->PID_calculate(desired_pose_vector_[1], actual_tcp_pose_[1], 0);
    position_z_controller_->PID_calculate(desired_pose_vector_[2], actual_tcp_pose_[2], 0);

    force_x_compensator_->PID_calculate(robot_task_->get_desired_force_torque()[0],current_ft_.force()[0],0);
    force_y_compensator_->PID_calculate(robot_task_->get_desired_force_torque()[1],current_ft_.force()[1],0);
    force_z_compensator_->PID_calculate(robot_task_->get_desired_force_torque()[2],current_ft_.force()[2],0);

    tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(force_x_compensator_->get_final_output(),force_y_compensator_->get_final_output(),force_z_compensator_->get_final_output()), EAA<>(0,0,0).toRotation3D());

    tf_modified_pose_ = tf_current_ * tf_tcp_desired_pose_;

    tf_modified_pose_.P() = tf_current_.P() - tf_modified_pose_.P();

    pid_compensation_[0] = position_x_controller_->get_final_output();
    pid_compensation_[1] = position_y_controller_->get_final_output();
    pid_compensation_[2] = position_z_controller_->get_final_output();
    pid_compensation_[3] = (tf_modified_pose_.P())[0];
    pid_compensation_[4] = (tf_modified_pose_.P())[1];
    pid_compensation_[5] = (tf_modified_pose_.P())[2];


    filtered_tcp_ft_data_[0] = current_ft_.force()[0];
    filtered_tcp_ft_data_[1] = current_ft_.force()[1];
    filtered_tcp_ft_data_[2] = current_ft_.force()[2];
    filtered_tcp_ft_data_[3] = current_ft_.torque()[0];
    filtered_tcp_ft_data_[4] = current_ft_.torque()[1];
    filtered_tcp_ft_data_[5] = current_ft_.torque()[2];

    compensated_pose_vector_[0] = actual_tcp_pose_[0] + position_x_controller_->get_final_output()+(tf_modified_pose_.P())[0];
    compensated_pose_vector_[1] = actual_tcp_pose_[1] + position_y_controller_->get_final_output()+(tf_modified_pose_.P())[1];
    compensated_pose_vector_[2] = actual_tcp_pose_[2] + position_z_controller_->get_final_output()+(tf_modified_pose_.P())[2];

    compensated_pose_vector_[3] = desired_pose_vector_[3]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector_[4] = desired_pose_vector_[4]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector_[5] = desired_pose_vector_[5]; //+ force_x_compensator->get_final_output();
  }
  else
  {

    position_x_controller_->PID_calculate(desired_pose_vector_[0], compensated_pose_vector_[0], 0);
    position_y_controller_->PID_calculate(desired_pose_vector_[1], compensated_pose_vector_[1], 0);
    position_z_controller_->PID_calculate(desired_pose_vector_[2], compensated_pose_vector_[2], 0);

    //contact_check_ = ros_state->get_test();

    pid_compensation_[0] = position_x_controller_->get_final_output();
    pid_compensation_[1] = position_y_controller_->get_final_output();
    pid_compensation_[2] = position_z_controller_->get_final_output();
    pid_compensation_[3] = 0;
    pid_compensation_[4] = 0;
    pid_compensation_[5] = 0;

    //cout << pid_compensation << endl;

    compensated_pose_vector_[0] = compensated_pose_vector_[0] + pid_compensation_[0]; //+ ros_state->get_rl_action()[0];
    compensated_pose_vector_[1] = compensated_pose_vector_[1] + pid_compensation_[1]; //+ ros_state->get_rl_action()[1];
    compensated_pose_vector_[2] = compensated_pose_vector_[2] + pid_compensation_[2]; //+ ros_state->get_rl_action()[2];

    compensated_pose_vector_[3] = desired_pose_vector_[3]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector_[4] = desired_pose_vector_[4]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector_[5] = desired_pose_vector_[5]; //+ force_x_compensator->get_final_output();


    robot_task_->set_current_pose_eaa(compensated_pose_vector_[0], compensated_pose_vector_[1], compensated_pose_vector_[2],compensated_pose_vector_[3], compensated_pose_vector_[4], compensated_pose_vector_[5]);
    tf_current_ = Transform3D<> (Vector3D<>(compensated_pose_vector_[0], compensated_pose_vector_[1], compensated_pose_vector_[2]), EAA<>(compensated_pose_vector_[3], compensated_pose_vector_[4], compensated_pose_vector_[5]).toRotation3D());
  }

  error_ee_pose_[0] = position_x_controller_->get_error();
  error_ee_pose_[1] = position_y_controller_->get_error();
  error_ee_pose_[2] = position_z_controller_->get_error();
  error_ee_pose_[3] = 0; //position_x_controller->get_error();
  error_ee_pose_[4] = 0; //position_x_controller->get_error();
  error_ee_pose_[5] = 0; //position_x_controller->get_error();


  tf_desired_ = Transform3D<> (Vector3D<>(compensated_pose_vector_[0], compensated_pose_vector_[1], compensated_pose_vector_[2]),
      EAA<>(compensated_pose_vector_[3], compensated_pose_vector_[4], compensated_pose_vector_[5]).toRotation3D());

  //solve ik problem
  solutions_ = solver_->solve(tf_desired_, state_);

  //	for(std::size_t i = 0; i < solutions_.size(); i++) {
  //		std::cout << i << " : " << solutions_[i] << std::endl;
  //	}

  for(int num = 0; num <6 ; num ++)
  {
    compensated_q_[num] = solutions_[preferred_solution_number_].toStdVector()[num];
  }

  //from covid - robot
  for (unsigned int num = 0; num < 6; num ++)
  {
    // LOG_DEBUG("[%s]", "Wrapping for joint");
    const double diffOrig = fabs(current_q_[num] - compensated_q_[num]);
    // LOG_DEBUG("[diffOrig: %e , %e]", (solutions[i][j])/rw::math::Pi*180.0, diffOrig );

    const double diffAdd = fabs(current_q_[num] - (compensated_q_[num] + 2 * rw::math::Pi));
    // LOG_DEBUG("[diffAdd: %e , %e]", (solutions[i][j]+2*rw::math::Pi)/rw::math::Pi*180.0, diffAdd );

    const double diffSub = fabs(current_q_[num] - (compensated_q_[num] - 2 * rw::math::Pi));
    // LOG_DEBUG("[diffSub: %e , %e]", (solutions[i][j]-2*rw::math::Pi)/rw::math::Pi*180.0, diffSub );

    if (diffAdd < diffOrig && diffAdd < diffSub)
    {
      compensated_q_[num] += 2 * rw::math::Pi;
    }
    else if (diffSub < diffOrig && diffSub < diffAdd)
    {
      compensated_q_[num] -= 2 * rw::math::Pi;
    }
  }

  //check velocity
  for(int num = 0; num <6 ; num ++)
  {
    if(fabs((compensated_q_[num] - current_q_[num])/control_time_) > 270*DEGREE2RADIAN)
    {
      std::cout << robot_name_ << "::" << num << "::" << fabs((compensated_q_[num] - current_q_[num])/control_time_) << std::endl;
      std::cout << COLOR_RED_BOLD << "Robot speed is so FAST" << COLOR_RESET << std::endl;
      joint_vel_limits_ = true;
      control_check_ = false;
    }
  }

  //send command in joint space to ur robot or gazebo
  if(!joint_vel_limits_)
  {
    if(!gazebo_check_)
    {
      control_check_ = rtde_control_->servoJ(compensated_q_,0,0,control_time_,0.03,2000);
    }
    for(int num = 0; num <6 ; num ++)
    {
      current_q_[num] = compensated_q_[num];
    }
  }
  else
  {
    control_check_ = false;
  }

  time_count_ += control_time_;
  //data log save
  data_log_->set_time_count(time_count_);
  data_log_->set_data_getActualQ(joint_positions_);
  data_log_->set_data_getActualTCPPose(actual_tcp_pose_);
  data_log_->set_data_getTargetTCPPose(target_tcp_pose_);
  data_log_->set_data_getActualTCPForceTorque(raw_ft_data_);
  data_log_->set_data_getActualToolAccelerometer(acutal_tcp_acc_);
  data_log_->set_data_getFilteredTCPForceTorque(filtered_tcp_ft_data_);
  data_log_->set_data_getContactedForceTorque(contacted_ft_data_);
  data_log_->set_data_getPidCompensation(pid_compensation_);
  data_log_->set_data_getDesiredTCPPose(desired_pose_vector_);
  data_log_->set_data_getBeltPosition(data_current_belt_);
  data_log_->set_data_getDesiredBeltPosition(data_desired_belt_);
  data_log_->set_data_new_line();

  return control_check_;
}
void TaskRobot::parse_init_data_(const std::string &path)
{
  YAML::Node doc; //
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str()); //

  }catch(const std::exception& e) //
  {
    cout << "Fail to load yaml file!" << endl;
    return;
  }

  tool_mass_= doc["tool_mass"].as<double>();
  //setting up control time and mass of tool
  tool_estimation_ ->set_parameters(control_time_, tool_mass_);

  preferred_solution_number_ =  doc["preferred_solution_number"].as<double>();
  YAML::Node initial_joint_states = doc["initial_joint_states"];
  YAML::Node bigger_pulley_bearing_position_node = doc["bigger_pulley_bearing_position"];
  std::vector<double> bigger_pulley_bearing_position;

  for(int num = 0; num < 6; num ++)
  {
    bigger_pulley_bearing_position.push_back(bigger_pulley_bearing_position_node[num].as<double>());
  }

  tf_base_to_bearing_ = Transform3D<> (Vector3D<>(bigger_pulley_bearing_position[0], bigger_pulley_bearing_position[1], bigger_pulley_bearing_position[2]), EAA<>(bigger_pulley_bearing_position[3], bigger_pulley_bearing_position[4], bigger_pulley_bearing_position[5]).toRotation3D());

  bigger_pulley_bearing_position.clear();

  force_controller_gain_.x_kp = doc["f_x_kp"].as<double>();
  force_controller_gain_.x_ki = doc["f_x_ki"].as<double>();
  force_controller_gain_.x_kd = doc["f_x_kd"].as<double>();
  force_controller_gain_.y_kp = doc["f_y_kp"].as<double>();
  force_controller_gain_.y_ki = doc["f_y_ki"].as<double>();
  force_controller_gain_.y_kd = doc["f_y_kd"].as<double>();
  force_controller_gain_.z_kp = doc["f_z_kp"].as<double>();
  force_controller_gain_.z_ki = doc["f_z_ki"].as<double>();
  force_controller_gain_.z_kd = doc["f_z_kd"].as<double>();
  force_controller_gain_.eaa_x_kp = doc["f_eaa_x_kp"].as<double>();
  force_controller_gain_.eaa_x_ki = doc["f_eaa_x_ki"].as<double>();
  force_controller_gain_.eaa_x_kd = doc["f_eaa_x_kd"].as<double>();
  force_controller_gain_.eaa_y_kp = doc["f_eaa_y_kp"].as<double>();
  force_controller_gain_.eaa_y_ki = doc["f_eaa_y_ki"].as<double>();
  force_controller_gain_.eaa_y_kd = doc["f_eaa_y_kd"].as<double>();
  force_controller_gain_.eaa_z_kp = doc["f_eaa_z_kp"].as<double>();
  force_controller_gain_.eaa_z_ki = doc["f_eaa_z_ki"].as<double>();
  force_controller_gain_.eaa_z_kd = doc["f_eaa_z_kd"].as<double>();

  position_controller_gain_.x_kp = doc["p_x_kp"].as<double>();
  position_controller_gain_.x_ki = doc["p_x_ki"].as<double>();
  position_controller_gain_.x_kd = doc["p_x_kd"].as<double>();
  position_controller_gain_.y_kp = doc["p_y_kp"].as<double>();
  position_controller_gain_.y_ki = doc["p_y_ki"].as<double>();
  position_controller_gain_.y_kd = doc["p_y_kd"].as<double>();
  position_controller_gain_.z_kp = doc["p_z_kp"].as<double>();
  position_controller_gain_.z_ki = doc["p_z_ki"].as<double>();
  position_controller_gain_.z_kd = doc["p_z_kd"].as<double>();
  position_controller_gain_.eaa_x_kp = doc["p_eaa_x_kp"].as<double>();
  position_controller_gain_.eaa_x_ki = doc["p_eaa_x_ki"].as<double>();
  position_controller_gain_.eaa_x_kd = doc["p_eaa_x_kd"].as<double>();
  position_controller_gain_.eaa_y_kp = doc["p_eaa_y_kp"].as<double>();
  position_controller_gain_.eaa_y_ki = doc["p_eaa_y_ki"].as<double>();
  position_controller_gain_.eaa_y_kd = doc["p_eaa_y_kd"].as<double>();
  position_controller_gain_.eaa_z_kp = doc["p_eaa_z_kp"].as<double>();
  position_controller_gain_.eaa_z_ki = doc["p_eaa_z_ki"].as<double>();
  position_controller_gain_.eaa_z_kd = doc["p_eaa_z_kd"].as<double>();

  current_q_[0] = initial_joint_states[0].as<double>();
  current_q_[1] = initial_joint_states[1].as<double>();
  current_q_[2] = initial_joint_states[2].as<double>();

  current_q_[3] = initial_joint_states[3].as<double>();
  current_q_[4] = initial_joint_states[4].as<double>();
  current_q_[5] = initial_joint_states[5].as<double>();
}
void TaskRobot::set_force_controller_x_gain(double kp,double ki,double kd)
{
  force_controller_gain_.x_kp = kp;
  force_controller_gain_.x_ki = ki;
  force_controller_gain_.x_kd = kd;
}
void TaskRobot::set_force_controller_y_gain(double kp,double ki,double kd)
{
  force_controller_gain_.y_kp = kp;
  force_controller_gain_.y_ki = ki;
  force_controller_gain_.y_kd = kd;
}
void TaskRobot::set_force_controller_z_gain(double kp,double ki,double kd)
{
  force_controller_gain_.z_kp = kp;
  force_controller_gain_.z_ki = ki;
  force_controller_gain_.z_kd = kd;
}
void TaskRobot::set_force_controller_eaa_x_gain(double kp,double ki,double kd)
{
  force_controller_gain_.eaa_x_kp = kp;
  force_controller_gain_.eaa_x_ki = ki;
  force_controller_gain_.eaa_x_kd = kd;
}
void TaskRobot::set_force_controller_eaa_y_gain(double kp,double ki,double kd)
{
  force_controller_gain_.eaa_y_kp = kp;
  force_controller_gain_.eaa_y_ki = ki;
  force_controller_gain_.eaa_y_kd = kd;
}
void TaskRobot::set_force_controller_eaa_z_gain(double kp,double ki,double kd)
{
  force_controller_gain_.eaa_z_kp = kp;
  force_controller_gain_.eaa_z_ki = ki;
  force_controller_gain_.eaa_z_kd = kd;
}
void TaskRobot::set_position_controller_x_gain(double kp,double ki,double kd)
{
  position_controller_gain_.x_kp = kp;
  position_controller_gain_.x_ki = ki;
  position_controller_gain_.x_kd = kd;
}
void TaskRobot::set_position_controller_y_gain(double kp,double ki,double kd)
{
  position_controller_gain_.y_kp = kp;
  position_controller_gain_.y_ki = ki;
  position_controller_gain_.y_kd = kd;
}
void TaskRobot::set_position_controller_z_gain(double kp,double ki,double kd)
{
  position_controller_gain_.z_kp = kp;
  position_controller_gain_.z_ki = ki;
  position_controller_gain_.z_kd = kd;
}
void TaskRobot::set_position_controller_eaa_x_gain(double kp,double ki,double kd)
{
  position_controller_gain_.eaa_x_kp = kp;
  position_controller_gain_.eaa_x_ki = ki;
  position_controller_gain_.eaa_x_kd = kd;
}
void TaskRobot::set_position_controller_eaa_y_gain(double kp,double ki,double kd)
{
  position_controller_gain_.eaa_y_kp = kp;
  position_controller_gain_.eaa_y_ki = ki;
  position_controller_gain_.eaa_y_kd = kd;
}
void TaskRobot::set_position_controller_eaa_z_gain(double kp,double ki,double kd)
{
  position_controller_gain_.eaa_z_kp = kp;
  position_controller_gain_.eaa_z_ki = ki;
  position_controller_gain_.eaa_z_kd = kd;
}
void TaskRobot::set_robust_value(double robust_value)
{
  belt_robust_value_ = robust_value;
}
void TaskRobot::set_tf_static_robot(rw::math::Transform3D<> tf_base_to_staric_robot, rw::math::Transform3D<> tf_base_to_bearing_static_robot)
{
  tf_base_to_static_robot_ = tf_base_to_staric_robot;
  tf_base_to_bearing_static_robot_ = tf_base_to_bearing_static_robot;
}
void TaskRobot::set_belt_change_values(double x, double y, double z)
{
  change_x_ = x;
  change_y_ = y;
  change_z_ = z;
}
std::vector<double> TaskRobot::get_raw_ft_data_()
{
  return raw_ft_data_;
}
std::vector<double> TaskRobot::get_contacted_ft_data_()
{
  return contacted_ft_data_;
}
std::vector<double> TaskRobot::get_error_ee_pose_()
{
  return error_ee_pose_;
}
std::vector<double> TaskRobot::get_actual_tcp_speed_()
{
  return actual_tcp_speed_;
}
std::vector<double> TaskRobot::get_current_q_()
{
  return current_q_;
}
rw::math::Transform3D<> TaskRobot::get_tf_current_()
{
  return tf_current_;
}
rw::math::Transform3D<> TaskRobot::get_tf_base_to_bearing_()
{
  return tf_base_to_bearing_;
}
void TaskRobot::terminate_robot()
{
  if(!gazebo_check_)
  {
    rtde_control_->servoStop();
    rtde_control_->stopScript();
  }
}
void TaskRobot::terminate_data_log()
{
  data_log_->save_file();
}



