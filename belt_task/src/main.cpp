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

  RT_TASK *curtask;
  RT_TASK_INFO curtaskinfo;
  RTIME tstart;

  curtask = rt_task_self();
  rt_task_inquire(curtask, &curtaskinfo);

  printf("Starting task %s with period of %f ms ....\n", curtaskinfo.name, control_time*1000);
  //Make the task periodic with a specified loop period
  rt_task_set_periodic(NULL, TM_NOW, LOOP_PERIOD);
  tstart = rt_timer_read();

  if(!gazebo_check)
  {
    std::cout << COLOR_GREEN_BOLD << "Real robot start " << COLOR_RESET << std::endl;
    //getting sensor values sensor filter
    //      acutal_tcp_acc  = rtde_receive->getActualToolAccelerometer();
    //      actual_tcp_pose = rtde_receive->getActualTCPPose();
    //
    // data types will be changed
    //      for(int var = 0; var < 6; var ++)
    //      {
    //        raw_force_torque_data_matrix(var,0) = raw_ft_data[var];
    //      }
    //      tool_acc_data_matrix(0,0) = -acutal_tcp_acc[0];
    //      tool_acc_data_matrix(1,0) = -acutal_tcp_acc[1];
    //      tool_acc_data_matrix(2,0) = -acutal_tcp_acc[2];
    //
    //      tf_current = Transform3D<> (Vector3D<>(actual_tcp_pose[0], actual_tcp_pose[1], actual_tcp_pose[2]), EAA<>(actual_tcp_pose[3], actual_tcp_pose[4], actual_tcp_pose[5]).toRotation3D());
    tf_current = Transform3D<> (Vector3D<>(compensated_pose_vector[0], compensated_pose_vector[1], compensated_pose_vector[2]), EAA<>(compensated_pose_vector[3], compensated_pose_vector[4], compensated_pose_vector[5]).toRotation3D());

    for(int num_row = 0; num_row < 4; num_row ++)
    {
      for(int num_col = 0; num_col < 4; num_col ++)
      {
        tf_current_matrix(num_row,num_col) = tf_current(num_row,num_col);
      }
    }
    tool_estimation->set_orientation_data(tf_current_matrix);
    tool_estimation->set_gravity_input_data(tool_acc_data_matrix.block(0,0,3,1));

    cout << tool_acc_data_matrix.block(0,0,3,1) << endl;
  }
  else
  {
    std::cout << COLOR_GREEN_BOLD << "Gazebo robot start " << COLOR_RESET << std::endl;
  }

  while(1)
  {
    static int sum_count = 0;
    static double previous_t = 0.0;
    static double sum_delayed_time = 0.0;
    static double aver_delayed_time = 0.0;

    tstart = rt_timer_read();

    sum_delayed_time += ((rt_timer_read() - tstart)/1000000.0  - previous_t) - control_time*1000;
    if(sum_count%1500 == 0 && sum_count != 0) // every 3 seconds,
    {
      aver_delayed_time = sum_delayed_time/sum_count;
      //printf("Average delayed time in 3s: %.5f ms\n",aver_delayed_time);
      sum_delayed_time = 0;
      sum_count = 0;
    }
    previous_t = (rt_timer_read() - tstart)/1000000.0;
    time_count += 0.002;
    sum_count += 1;
    //do something

    //run motion trajectory task_motion
    ur10e_task->set_current_pose_eaa(compensated_pose_vector[0], compensated_pose_vector[1], compensated_pose_vector[2],compensated_pose_vector[3], compensated_pose_vector[4], compensated_pose_vector[5]);
    ur10e_task->run_task_motion();
    ur10e_task->generate_trajectory();

    //motion reference
    desired_pose_vector = ur10e_task->get_current_pose();
    desired_force_torque_vector = ur10e_task->get_desired_force_torque();

    if(!gazebo_check)
    {
      //getting sensor values sensor filter
      //raw_ft_data     = rtde_receive->getActualTCPForce();
      //acutal_tcp_acc  = rtde_receive->getActualToolAccelerometer();
      //actual_tcp_pose = rtde_receive->getActualTCPPose();
      //joint_positions = rtde_receive->getActualQ();
      //
      // data types will be changed
      //for(int var = 0; var < 6; var ++)
      //{
      //  raw_force_torque_data_matrix(var,0) = raw_ft_data[var];
      //}
      //tool_acc_data_matrix(0,0) = -acutal_tcp_acc[0];
      //tool_acc_data_matrix(1,0) = -acutal_tcp_acc[1];
      //tool_acc_data_matrix(2,0) = -acutal_tcp_acc[2];
      //
      //tf_current = Transform3D<> (Vector3D<>(actual_tcp_pose[0], actual_tcp_pose[1], actual_tcp_pose[2]), EAA<>(actual_tcp_pose[3], actual_tcp_pose[4], actual_tcp_pose[5]).toRotation3D());

      tf_current = Transform3D<> (Vector3D<>(compensated_pose_vector[0], compensated_pose_vector[1], compensated_pose_vector[2]), EAA<>(compensated_pose_vector[3], compensated_pose_vector[4], compensated_pose_vector[5]).toRotation3D());

      for(int num_row = 0; num_row < 4; num_row ++)
      {
        for(int num_col = 0; num_col < 4; num_col ++)
        {
          tf_current_matrix(num_row,num_col) = tf_current(num_row,num_col);
        }
      }
      //force torque sensor filtering
      tool_estimation->set_orientation_data(tf_current_matrix);
      tool_acc_data_matrix = tf_current_matrix*tool_acc_data_matrix;

      contacted_ft_data = tool_estimation->get_estimated_force(raw_force_torque_data_matrix, tool_acc_data_matrix.block(0,0,3,1));

      ////controller for force compensation
      force_x_compensator->set_pid_gain(f_kp,f_ki,f_kd);
      force_y_compensator->set_pid_gain(f_kp,f_ki,f_kd);
      force_z_compensator->set_pid_gain(f_kp,f_ki,f_kd);

      force_x_compensator->PID_calculate(ur10e_task->get_desired_force_torque()[0],contacted_ft_data[0]);
      force_y_compensator->PID_calculate(ur10e_task->get_desired_force_torque()[1],contacted_ft_data[1]);
      force_z_compensator->PID_calculate(ur10e_task->get_desired_force_torque()[2],contacted_ft_data[2]);

      current_ft = Wrench6D<> (contacted_ft_data[0], contacted_ft_data[1], contacted_ft_data[2], contacted_ft_data[3], contacted_ft_data[4], contacted_ft_data[5]);

      tf_tcp_current_force = (tf_current.R()).inverse()*current_ft;

      filtered_ft_data[0] = tf_tcp_current_force.force()[0];
      filtered_ft_data[1] = tf_tcp_current_force.force()[1];
      filtered_ft_data[2] = tf_tcp_current_force.force()[2];
      filtered_ft_data[3] = tf_tcp_current_force.torque()[0];
      filtered_ft_data[4] = tf_tcp_current_force.torque()[1];
      filtered_ft_data[5] = tf_tcp_current_force.torque()[2];
      //
    }

    compensated_pose_vector[0] = desired_pose_vector[0]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector[1] = desired_pose_vector[1]; //+ force_y_compensator->get_final_output();
    compensated_pose_vector[2] = desired_pose_vector[2]; //+ force_x_compensator->get_final_output();

    compensated_pose_vector[3] = desired_pose_vector[3]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector[4] = desired_pose_vector[4]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector[5] = desired_pose_vector[5]; //+ force_x_compensator->get_final_output();

    tf_desired = Transform3D<> (Vector3D<>(compensated_pose_vector[0], compensated_pose_vector[1], compensated_pose_vector[2]),
        EAA<>(compensated_pose_vector[3], compensated_pose_vector[4], compensated_pose_vector[5]).toRotation3D());

    //solve ik problem
//    solutions = solver.solve(tf_desired, state);

    //from covid - robot
//    for (unsigned int num = 0; num < 6; num ++)
//    {
//      // LOG_DEBUG("[%s]", "Wrapping for joint");
//      const double diffOrig = fabs(current_q[num] - solutions[1][num]);
//      // LOG_DEBUG("[diffOrig: %e , %e]", (solutions[i][j])/rw::math::Pi*180.0, diffOrig );
//
//      const double diffAdd = fabs(current_q[num] - (solutions[1][num] + 2 * rw::math::Pi));
//      // LOG_DEBUG("[diffAdd: %e , %e]", (solutions[i][j]+2*rw::math::Pi)/rw::math::Pi*180.0, diffAdd );
//
//      const double diffSub = fabs(current_q[num] - (solutions[1][num] - 2 * rw::math::Pi));
//      // LOG_DEBUG("[diffSub: %e , %e]", (solutions[i][j]-2*rw::math::Pi)/rw::math::Pi*180.0, diffSub );
//
//      if (diffAdd < diffOrig && diffAdd < diffSub)
//      {
//        solutions[1][num] += 2 * rw::math::Pi;
//      }
//      else if (diffSub < diffOrig && diffSub < diffAdd)
//      {
//        solutions[1][num] -= 2 * rw::math::Pi;
//      }
//    }
//
//    //check velocity
//    for(int num = 0; num <6 ; num ++)
//    {
//      if(fabs((solutions[1].toStdVector()[num] - current_q[num])/control_time) > 45*DEGREE2RADIAN)
//      {
//        cout << "::" << num << "::" << fabs((solutions[1].toStdVector()[num] - current_q[num])/control_time) << endl;
//        std::cout << COLOR_RED_BOLD << "Robot speed is so FAST" << COLOR_RESET << std::endl;
//        joint_vel_limits = true;
//      }
//    }
//
//    //send command in joint space to ur robot or gazebo
//    if(!joint_vel_limits)
//    {
//      if(!gazebo_check)
//      {
//        //rtde_control->servoJ(solutions[1].toStdVector(),0,0,0.002,0.04,100);
//      }
//      else
//      {
//        ros_state->send_gazebo_command(solutions[1].toStdVector());
//      }
//      for(int num = 0; num <6 ; num ++)
//      {
//        current_q[num] = solutions[1].toStdVector()[num];
//      }
//    }

    data_log->set_time_count(time_count);
    data_log->set_data_getActualQ(joint_positions);
    data_log->set_data_getActualTCPPose(actual_tcp_pose);
    data_log->set_data_getTargetTCPPose(target_tcp_pose);
    data_log->set_data_getActualTCPForceTorque(raw_ft_data);
    data_log->set_data_getActualToolAccelerometer(acutal_tcp_acc);
    data_log->set_data_getFilteredForceTorque(filtered_ft_data);
    data_log->set_data_getContactedForceTorque(contacted_ft_data);
    data_log->set_data_new_line();

    previous_t = (rt_timer_read() - tstart)/1000000.0;

    cout << previous_t << endl;
    rt_task_wait_period(NULL);
  }

}
void initialize()
{
  //simulation check
  gazebo_check = true;
  control_time = 0.002;

  //data log
  data_log = std::make_shared<DataLogging>();
  data_log->initialize();

  //tool_estimation
  tool_estimation = std::make_shared<ToolEstimation>();
  tool_estimation->initialize();

  // fifth order traj
  ur10e_traj = std::make_shared<EndEffectorTraj>();
  ur10e_task = std::make_shared<TaskMotion>();

  //setting up control time and mass of tool
  tool_estimation ->set_parameters(control_time, 1.75);
  ur10e_traj->set_control_time(control_time);
  ur10e_task->initialize(control_time);

  //control
  force_x_compensator = std::make_shared<PID_function>(control_time, 0.0025, -0.0025, 0, 0, 0, 0.00001, -0.00001);
  force_y_compensator = std::make_shared<PID_function>(control_time, 0.0025, -0.0025, 0, 0, 0, 0.00001, -0.00001);
  force_z_compensator = std::make_shared<PID_function>(control_time, 0.0025, -0.0025, 0, 0, 0, 0.00001, -0.00001);

  //robot A
  ur10e_task->set_initial_pose(-0.6649675947900577, 0.2054059150102794, 0.25997552421325143, -0.7298511759689527, -1.7577922996928013, 1.760382318187891); // set to be robot initial values
  ur10e_task->set_initial_pose_eaa(-0.6649675947900577, 0.2054059150102794, 0.25997552421325143, -0.7298511759689527, -1.7577922996928013, 1.760382318187891); // set to be robot initial values

  desired_pose_vector[0] = -0.6649675947900577;
  desired_pose_vector[1] = 0.2054059150102794 ;
  desired_pose_vector[2] = 0.25997552421325143;
  desired_pose_vector[3] = -0.7298511759689527;
  desired_pose_vector[4] = -1.7577922996928013;
  desired_pose_vector[5] = 1.760382318187891  ;

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

  tf_current_matrix.resize(4,4);
  tf_current_matrix.fill(0);

  raw_force_torque_data_matrix.resize(6,1);
  raw_force_torque_data_matrix.fill(0);
  tool_acc_data_matrix.resize(4,1);
  tool_acc_data_matrix.fill(0);

  task_command = "";

  //load motion data
  std::string path_ = "/home/yik/sdu_ws/SDU-Control-Tasks/belt_task/config/motion/initialize_belt_task.yaml";
  ur10e_task->load_task_motion(path_,"initialize_belt_task");
  path_ = "/home/yik/sdu_ws/SDU-Control-Tasks/belt_task/config/motion/initialize.yaml";
  ur10e_task->load_task_motion(path_,"initialize");
  path_ = "/home/yik/sdu_ws/SDU-Control-Tasks/belt_task/config/motion/tcp_belt_task.yaml";
  ur10e_task->set_initial_pose_eaa(-0.594295, 0.00668736, 0.244747, -0.7290212721930756, -1.7599986104134355, 1.7600122635046096); // set to be robot initial values
  ur10e_task->trans_tcp_to_base_motion(path_);
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
    const std::string robot_ip = "192.168.1.130"; // robot B

    //rtde_receive = std::make_shared<RTDEReceiveInterface>(robot_ip);
    //rtde_control = std::make_shared<RTDEControlInterface>(robot_ip);

    //rtde_control->zeroFtSensor();

    std::cout << COLOR_YELLOW_BOLD << "Robot connected to your program" << COLOR_RESET << std::endl;
    std::cout << COLOR_RED_BOLD << "Robot will move 2 seconds later" << COLOR_RESET << std::endl;
    usleep(2000000);
    //rtde_control->moveJ(current_Q,0.1,0.1); // move to initial pose
    std::cout << COLOR_RED_BOLD << "Send" << COLOR_RESET << std::endl;
    usleep(2000000);
  }
  std::cout << COLOR_GREEN << "All of things were initialized!" << COLOR_RESET << std::endl;

  //real time task
  char str[35];
  mlockall(MCL_CURRENT | MCL_FUTURE); //Lock the memory to avoid memory swapping for this program

  printf("Starting cyclic task...\n");
  sprintf(str, "Belt Task Start");
  rt_task_create(&loop_task, str, 0, 80, 0);//Create the real time task
  rt_task_start(&loop_task, &loop_task_proc, 0);//Since task starts in suspended mode, start task

  std::cout << COLOR_GREEN << "Real time task loop was created!" << COLOR_RESET << std::endl;

  pause();
  rt_task_delete(&loop_task);

  ros_state->shout_down_ros();
  data_log->save_file();
  usleep(3000000);
  if(!gazebo_check)
  {
    //rtde_control->servoStop();
  }
  return 0;
}
