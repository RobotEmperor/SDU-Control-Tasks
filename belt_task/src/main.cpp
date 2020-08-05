/*
 * main.cpp
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */
#include "belt_task.h"

void loop_task_proc(void *arg)
{
  if (wc.isNull())
    RW_THROW("WorkCell could not be loaded.");
  if (device.isNull())
    RW_THROW("UR10e device could not be found.");

  State state = wc->getDefaultState();
  ClosedFormIKSolverUR solver(device, state);
  solver.setCheckJointLimits(true);

  // RT_TASK *curtask;
  //  RT_TASK_INFO curtaskinfo;

  //
  //  curtask = rt_task_self();
  //  rt_task_inquire(curtask, &curtaskinfo);
  //
  //  printf("Starting task %s with period of %f ms ....\n", curtaskinfo.name, control_time*1000);
  //Make the task periodic with a specified loop period

  RTIME tstart;
  rt_task_set_periodic(NULL, TM_NOW, LOOP_PERIOD);
  tstart = rt_timer_read();

  if(!gazebo_check)
  {
    std::cout << COLOR_GREEN_BOLD << "Real robot start " << COLOR_RESET << std::endl;
    //getting sensor values sensor filter
    acutal_tcp_acc  = rtde_receive_a->getActualToolAccelerometer();
    actual_tcp_pose = rtde_receive_a->getActualTCPPose();

    tf_current = Transform3D<> (Vector3D<>(actual_tcp_pose[0], actual_tcp_pose[1], actual_tcp_pose[2]), EAA<>(actual_tcp_pose[3], actual_tcp_pose[4], actual_tcp_pose[5]).toRotation3D());

    tool_estimation->set_orientation_data(tf_current);
    tool_estimation->set_gravity_input_data(acutal_tcp_acc);

    cout << acutal_tcp_acc << endl;
  }
  else
  {
    std::cout << COLOR_GREEN_BOLD << "Gazebo robot start " << COLOR_RESET << std::endl;
    compensated_pose_vector[0] = -0.6649675947900577;
    compensated_pose_vector[1] = 0.2054059150102794;
    compensated_pose_vector[2] = 0.25997552421325143;

    compensated_pose_vector[3] = -0.7298511759689527;
    compensated_pose_vector[4] = -1.7577922996928013;
    compensated_pose_vector[5] = 1.760382318187891;
  }

  while(1)
  {
    static double previous_t = 0.0;
    tstart = rt_timer_read();
    time_count += control_time;

    ros_state->update_ros_data();

    //do something
    //check if task command is received or not
    //run motion trajectory task_motion

    //auto_task_motion goal
    if(!ros_state->get_task_command().compare("auto"))
    {
      static int previous_phases_ = 0;
      ur10e_task->auto_task_motion(contact_check);

      if(ur10e_task->get_phases_() == 1 && previous_phases_ == 2)
      {
        cout << "Zero ft sensor" << endl;
        tool_estimation->set_sensor_offset_value(raw_ft_data);
      }
      previous_phases_ = ur10e_task->get_phases_();
    }
    else
    {
      if(!ros_state->get_task_command().compare("set_point"))
      {
        static double set_point_motion_time_ = 0.0;
        set_point_motion_time_ = ros_state->get_set_point()[6];
        ur10e_task->tf_set_point_base(ros_state->get_set_point());
        set_point_vector = ur10e_task->get_set_point_base();
        ur10e_task->set_point(set_point_vector[0], set_point_vector[1], set_point_vector[2], set_point_vector[3], set_point_vector[4], set_point_vector[5], set_point_motion_time_);
        ur10e_task->generate_trajectory();
        ros_state->clear_task_command();
      }
      else
      {
        if(ros_state->get_task_command().compare(previous_task_command) != 0 && ros_state->get_task_command().compare("") != 0)
        {
          ur10e_task->clear_phase();
          ur10e_task->clear_task_motion();
          ur10e_task->change_motion(ros_state->get_task_command());
          contact_check = false;
        }
        ur10e_task->run_task_motion();
        ur10e_task->generate_trajectory();
      }
    }
    //motion reference
    desired_pose_vector = ur10e_task->get_current_pose();
    desired_force_torque_vector = ur10e_task->get_desired_force_torque();

    p_kp = ros_state->get_p_gain();
    p_ki = ros_state->get_i_gain();
    p_kd = ros_state->get_d_gain();

    //controller for pose controller
    position_x_controller->set_pid_gain(p_kp,p_ki,p_kd);
    position_y_controller->set_pid_gain(p_kp,p_ki,p_kd);
    position_z_controller->set_pid_gain(p_kp,p_ki,p_kd);

    f_kp = ros_state->get_force_p_gain();
    f_ki = ros_state->get_force_i_gain();
    f_kd = ros_state->get_force_d_gain();

    //controller for force compensation
    force_x_compensator->set_pid_gain(f_kp,f_ki,f_kd);
    force_y_compensator->set_pid_gain(f_kp,f_ki,f_kd);
    force_z_compensator->set_pid_gain(f_kp,f_ki,f_kd);

    if(!gazebo_check)
    {
      //getting sensor values sensor filter
      raw_ft_data     = rtde_receive_a->getActualTCPForce();
      acutal_tcp_acc  = rtde_receive_a->getActualToolAccelerometer();
      actual_tcp_pose = rtde_receive_a->getActualTCPPose();
      //joint_positions = rtde_receive_a->getActualQ();

      ur10e_task->set_current_pose_eaa(actual_tcp_pose[0], actual_tcp_pose[1], actual_tcp_pose[2],actual_tcp_pose[3], actual_tcp_pose[4], actual_tcp_pose[5]);

      tf_current = Transform3D<> (Vector3D<>(actual_tcp_pose[0], actual_tcp_pose[1], actual_tcp_pose[2]), EAA<>(actual_tcp_pose[3], actual_tcp_pose[4], actual_tcp_pose[5]).toRotation3D());

      //force torque sensor filtering
      tool_estimation->set_orientation_data(tf_current);
      tool_estimation->process_estimated_force(raw_ft_data, acutal_tcp_acc);

      contacted_ft_data = tool_estimation->get_contacted_force();
      current_ft = Wrench6D<> (contacted_ft_data[0], contacted_ft_data[1], contacted_ft_data[2], contacted_ft_data[3], contacted_ft_data[4], contacted_ft_data[5]);
      current_ft = (tf_current.R()).inverse()*current_ft;
      tf_current = Transform3D<> (Vector3D<>(actual_tcp_pose[0], actual_tcp_pose[1], actual_tcp_pose[2]), EAA<>(actual_tcp_pose[3], actual_tcp_pose[4], actual_tcp_pose[5]).toRotation3D());


      if(current_ft.force()[1] > 5  && !contact_check)
      {
        contact_check = 1;
        cout << "A rubber belt was inserted in a pulley" << endl;
      }

      position_x_controller->PID_calculate(desired_pose_vector[0], actual_tcp_pose[0], 0);
      position_y_controller->PID_calculate(desired_pose_vector[1], actual_tcp_pose[1], 0);
      position_z_controller->PID_calculate(desired_pose_vector[2], actual_tcp_pose[2], 0);

      force_x_compensator->PID_calculate(ur10e_task->get_desired_force_torque()[0],current_ft.force()[0],0);
      force_y_compensator->PID_calculate(ur10e_task->get_desired_force_torque()[1],current_ft.force()[1],0);
      force_z_compensator->PID_calculate(ur10e_task->get_desired_force_torque()[2],current_ft.force()[2],0);

      tf_tcp_desired_pose = Transform3D<> (Vector3D<>(force_x_compensator->get_final_output(),force_y_compensator->get_final_output(),force_z_compensator->get_final_output()), EAA<>(0,0,0).toRotation3D());

      tf_modified_pose = tf_current * tf_tcp_desired_pose;

      tf_modified_pose.P() = tf_current.P() - tf_modified_pose.P();

      pid_compensation[0] = position_x_controller->get_final_output();
      pid_compensation[1] = position_y_controller->get_final_output();
      pid_compensation[2] = position_z_controller->get_final_output();
      pid_compensation[3] = (tf_modified_pose.P())[0];
      pid_compensation[4] = (tf_modified_pose.P())[1];
      pid_compensation[5] = (tf_modified_pose.P())[2];


      filtered_tcp_ft_data[0] = current_ft.force()[0];
      filtered_tcp_ft_data[1] = current_ft.force()[1];
      filtered_tcp_ft_data[2] = current_ft.force()[2];
      filtered_tcp_ft_data[3] = current_ft.torque()[0];
      filtered_tcp_ft_data[4] = current_ft.torque()[1];
      filtered_tcp_ft_data[5] = current_ft.torque()[2];

      compensated_pose_vector[0] = actual_tcp_pose[0] + position_x_controller->get_final_output()+(tf_modified_pose.P())[0];
      compensated_pose_vector[1] = actual_tcp_pose[1] + position_y_controller->get_final_output()+(tf_modified_pose.P())[1];
      compensated_pose_vector[2] = actual_tcp_pose[2] + position_z_controller->get_final_output()+(tf_modified_pose.P())[2];

      compensated_pose_vector[3] = desired_pose_vector[3]; //+ force_x_compensator->get_final_output();
      compensated_pose_vector[4] = desired_pose_vector[4]; //+ force_x_compensator->get_final_output();
      compensated_pose_vector[5] = desired_pose_vector[5]; //+ force_x_compensator->get_final_output();
    }
    else
    {
      position_x_controller->PID_calculate(desired_pose_vector[0], compensated_pose_vector[0], 0);
      position_y_controller->PID_calculate(desired_pose_vector[1], compensated_pose_vector[1], 0);
      position_z_controller->PID_calculate(desired_pose_vector[2], compensated_pose_vector[2], 0);

      contact_check = ros_state->get_test();
      ur10e_task->set_current_pose_eaa(compensated_pose_vector[0], compensated_pose_vector[1], compensated_pose_vector[2],compensated_pose_vector[3], compensated_pose_vector[4], compensated_pose_vector[5]);

      pid_compensation[0] = position_x_controller->get_final_output();
      pid_compensation[1] = position_y_controller->get_final_output();
      pid_compensation[2] = position_z_controller->get_final_output();
      pid_compensation[3] = 0;
      pid_compensation[4] = 0;
      pid_compensation[5] = 0;

      //cout << pid_compensation << endl;

      compensated_pose_vector[0] = compensated_pose_vector[0] + pid_compensation[0];
      compensated_pose_vector[1] = compensated_pose_vector[1] + pid_compensation[1];
      compensated_pose_vector[2] = compensated_pose_vector[2] + pid_compensation[2];

      compensated_pose_vector[3] = desired_pose_vector[3]; //+ force_x_compensator->get_final_output();
      compensated_pose_vector[4] = desired_pose_vector[4]; //+ force_x_compensator->get_final_output();
      compensated_pose_vector[5] = desired_pose_vector[5]; //+ force_x_compensator->get_final_output();
    }


    tf_desired = Transform3D<> (Vector3D<>(compensated_pose_vector[0], compensated_pose_vector[1], compensated_pose_vector[2]),
        EAA<>(compensated_pose_vector[3], compensated_pose_vector[4], compensated_pose_vector[5]).toRotation3D());

    //solve ik problem

    solutions = solver.solve(tf_desired, state);

    //from covid - robot
    for (unsigned int num = 0; num < 6; num ++)
    {
      // LOG_DEBUG("[%s]", "Wrapping for joint");
      const double diffOrig = fabs(current_q[num] - solutions[1][num]);
      // LOG_DEBUG("[diffOrig: %e , %e]", (solutions[i][j])/rw::math::Pi*180.0, diffOrig );

      const double diffAdd = fabs(current_q[num] - (solutions[1][num] + 2 * rw::math::Pi));
      // LOG_DEBUG("[diffAdd: %e , %e]", (solutions[i][j]+2*rw::math::Pi)/rw::math::Pi*180.0, diffAdd );

      const double diffSub = fabs(current_q[num] - (solutions[1][num] - 2 * rw::math::Pi));
      // LOG_DEBUG("[diffSub: %e , %e]", (solutions[i][j]-2*rw::math::Pi)/rw::math::Pi*180.0, diffSub );

      if (diffAdd < diffOrig && diffAdd < diffSub)
      {
        solutions[1][num] += 2 * rw::math::Pi;
      }
      else if (diffSub < diffOrig && diffSub < diffAdd)
      {
        solutions[1][num] -= 2 * rw::math::Pi;
      }
    }

    //check velocity
    for(int num = 0; num <6 ; num ++)
    {
      if(fabs((solutions[1].toStdVector()[num] - current_q[num])/control_time) > 270*DEGREE2RADIAN)
      {
        cout << "::" << num << "::" << fabs((solutions[1].toStdVector()[num] - current_q[num])/control_time) << endl;
        std::cout << COLOR_RED_BOLD << "Robot speed is so FAST" << COLOR_RESET << std::endl;
        joint_vel_limits = true;
      }
    }

    //send command in joint space to ur robot or gazebo
    if(!joint_vel_limits)
    {
      if(!gazebo_check)
      {
        rtde_control_a->servoJ(solutions[1].toStdVector(),0,0,control_time,0.04,2000);

        //for test
        //ros_state->send_gazebo_command(solutions[1].toStdVector());
        //check if there is out of the real-time control
        previous_t = (rt_timer_read() - tstart)/1000000.0;
        if(previous_t > 2)
          cout << COLOR_RED_BOLD << "Exceed control time A "<< previous_t << COLOR_RESET << endl;
      }
      else
      {
        ros_state->send_gazebo_command(solutions[1].toStdVector());
      }
      for(int num = 0; num <6 ; num ++)
      {
        current_q[num] = solutions[1].toStdVector()[num];
      }
    }

    ros_state->send_raw_ft_data(raw_ft_data);
    ros_state->send_filtered_ft_data(contacted_ft_data);

    //data log save
    data_log->set_time_count(time_count);
    data_log->set_data_getActualQ(joint_positions);
    data_log->set_data_getActualTCPPose(actual_tcp_pose);
    data_log->set_data_getTargetTCPPose(target_tcp_pose);
    data_log->set_data_getActualTCPForceTorque(raw_ft_data);
    data_log->set_data_getActualToolAccelerometer(acutal_tcp_acc);
    data_log->set_data_getFilteredTCPForceTorque(filtered_tcp_ft_data);
    data_log->set_data_getContactedForceTorque(contacted_ft_data);
    data_log->set_data_getPidCompensation(pid_compensation);
    data_log->set_data_getDesiredTCPPose(desired_pose_vector);
    data_log->set_data_new_line();

    previous_task_command = ros_state->get_task_command();
    rt_task_wait_period(NULL);
  }
}
void initialize()
{
  std::string path_ = "/home/yik/sdu_ws/SDU-Control-Tasks/belt_task/config/initial_condition.yaml";

  //simulation check
  gazebo_check = true;
  control_time = 0.002;

  //data log
  data_log = std::make_shared<DataLogging>();
  data_log->initialize();

  //tool_estimation
  tool_estimation = std::make_shared<ToolEstimation>();
  tool_estimation->initialize();

  contact_check = false;

  //statistics
  statistics_math = std::make_shared<StatisticsMath>();

  // fifth order traj
  ur10e_traj = std::make_shared<EndEffectorTraj>();
  ur10e_task = std::make_shared<TaskMotion>();

  //setting up control time and mass of tool
  tool_estimation ->set_parameters(control_time, 1.75);
  ur10e_traj->set_control_time(control_time);
  ur10e_task->initialize(control_time, path_);

  //control
  force_x_compensator = std::make_shared<PID_function>(control_time, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);
  force_y_compensator = std::make_shared<PID_function>(control_time, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);
  force_z_compensator = std::make_shared<PID_function>(control_time, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);

  position_x_controller = std::make_shared<PID_function>(control_time, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);
  position_y_controller = std::make_shared<PID_function>(control_time, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);
  position_z_controller = std::make_shared<PID_function>(control_time, 0.0045, -0.0045, 0, 0, 0, 0.0000001, -0.0000001);

  //robot A
  desired_pose_vector = ur10e_task -> get_current_pose();

  // have to fix for automatic calculation
  current_q[0] = 2.6559;
  current_q[1] = -1.85396;
  current_q[2] = -2.03763;
  current_q[3] = -2.38718;
  current_q[4] = -2.84058;
  current_q[5] = -3.13698;

  joint_vel_limits = false;
  motion_time = 5;

  f_kp = 0;
  f_ki = 0;
  f_kd = 0;

  previous_task_command = "";

  //load motion data
  path_ = "/home/yik/sdu_ws/SDU-Control-Tasks/belt_task/config/motion/initialize_belt_task.yaml";
  ur10e_task->load_task_motion(path_,"initialize_belt_task");
  path_ = "/home/yik/sdu_ws/SDU-Control-Tasks/belt_task/config/motion/initialize.yaml";
  ur10e_task->load_task_motion(path_,"initialize");
  path_ = "/home/yik/sdu_ws/SDU-Control-Tasks/belt_task/config/motion/tcp_belt_task.yaml";
  ur10e_task->trans_tcp_to_base_motion(path_);

  exit_program = false;
}
void my_function(int sig)
{ // can be called asynchronously
  exit_program = true; // set flag
}
int main (int argc, char **argv)
{
  initialize();
  ros_state = std::make_shared<RosNode>(argc,argv,"Belt_Task");
  ros_state->initialize();

  std::cout << COLOR_YELLOW_BOLD << "Simulation On [ yes / no ]" << COLOR_RESET << std::endl;
  cin >> silmulation_on_off;

  if(!silmulation_on_off.compare("yes") || !silmulation_on_off.compare("y"))
    std::cout << COLOR_GREEN_BOLD << "Setting up Simulation " << COLOR_RESET << std::endl;
  else
  {
    std::cout << COLOR_GREEN_BOLD << "REAL Robot, Be careful to run:" << COLOR_RESET << std::endl;
    std::cout << COLOR_GREEN_BOLD << "Are you sure ? [yes / no]" << COLOR_RESET << std::endl;
    cin >> silmulation_on_off;
    if(silmulation_on_off.compare("y")!=0)
    {
      ros_state->shout_down_ros();
      data_log->save_file();
      std::cout << COLOR_RED_BOLD << "Terminate program" << COLOR_RESET << std::endl;
      return 0;
    }
    gazebo_check = false;
  }
  std::cout << COLOR_GREEN_BOLD << "Program Start:" << COLOR_RESET << std::endl;

  usleep(3000000);

  ros_state->update_ros_data();

  wc = WorkCellLoader::Factory::load(WC_FILE);
  device = wc->findDevice<SerialDevice>("UR10e");

  if(gazebo_check)
  {
    ros_state->send_gazebo_command(current_q);
  }
  else
  {
    robot_ip_a = "192.168.1.130";
    //robot_ip_b = "192.168.1.129";

    rtde_receive_a = std::make_shared<RTDEReceiveInterface>(robot_ip_a);
    rtde_control_a = std::make_shared<RTDEControlInterface>(robot_ip_a);
    rtde_control_a->zeroFtSensor();

    std::cout << COLOR_YELLOW_BOLD << "Robot A connected to your program" << COLOR_RESET << std::endl;
    std::cout << COLOR_RED_BOLD << "Robot A will move 2 seconds later" << COLOR_RESET << std::endl;
    usleep(2000000);
    rtde_control_a->moveJ(current_q,0.1,0.1); // move to initial pose
    std::cout << COLOR_RED_BOLD << "Send" << COLOR_RESET << std::endl;
    usleep(3000000);
  }
  std::cout << COLOR_GREEN << "All of things were initialized!" << COLOR_RESET << std::endl;

  //real time task
  char str[35];
  mlockall(MCL_CURRENT | MCL_FUTURE); //Lock the memory to avoid memory swapping for this program

  printf("Starting cyclic task...\n");
  sprintf(str, "Belt Task Start");
  rt_task_create(&loop_task, str, 0, 99, 0);//Create the real time task
  rt_task_start(&loop_task, &loop_task_proc, 0);//Since task starts in suspended mode, start task

  std::cout << COLOR_GREEN << "Real time task loop was created!" << COLOR_RESET << std::endl;

  signal(SIGINT, my_function);
  while(!exit_program)
  {
  }
  rt_task_delete(&loop_task);

  usleep(3000000);

  ros_state->shout_down_ros();
  data_log->save_file();
  usleep(3000000);
  if(!gazebo_check)
  {
    rtde_control_a->servoStop();
  }
  return 0;
}
