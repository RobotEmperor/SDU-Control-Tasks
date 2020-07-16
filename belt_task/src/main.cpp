/*
 * main.cpp
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */
#include "belt_task.h"

void loop_task_proc(void *arg)
{
  RT_TASK *curtask;
  RT_TASK_INFO curtaskinfo;

  curtask = rt_task_self();
  rt_task_inquire(curtask, &curtaskinfo);

  printf("Starting task %s with period of %f ms ....\n", curtaskinfo.name, control_time*1000);

  RTIME tstart;
  rt_task_set_periodic(NULL, TM_NOW, LOOP_PERIOD);
  tstart = rt_timer_read();

  if(!gazebo_check)
  {
    robot_A->init_accelerometer();
  }
  else
  {
    robot_A->simulation_initialize();
  }

  while(1)
  {
    static double previous_t = 0.0;
    tstart = rt_timer_read();

    ros_state->update_ros_data();

    robot_A->set_current_task_motion(ros_state->get_task_command());
    robot_A->set_position_gain(ros_state->get_p_gain(), ros_state->get_i_gain(), ros_state->get_d_gain());
    robot_A->set_force_gain(ros_state->get_force_p_gain(), ros_state->get_force_i_gain(), ros_state->get_force_d_gain());
    robot_A->set_test_contact(ros_state->get_test());


    robot_A->controller();

    if(gazebo_check)
    {
      ros_state->send_gazebo_command(robot_A->get_desired_q_());
    }

    ros_state->send_raw_ft_data(robot_A->get_raw_ft_data());
    ros_state->send_filtered_ft_data(robot_A->get_contacted_ft_data());

    //data log save
    data_log->set_time_count(robot_A->get_time_count());
    data_log->set_data_getActualQ(robot_A->get_actual_joint_positions());
    data_log->set_data_getActualTCPPose(robot_A->get_actual_tcp_pose());
    data_log->set_data_getTargetTCPPose(robot_A->get_target_tcp_pose());
    data_log->set_data_getActualTCPForceTorque(robot_A->get_raw_ft_data());
    data_log->set_data_getActualToolAccelerometer(robot_A->get_acutal_tcp_acc());
    data_log->set_data_getFilteredTCPForceTorque(robot_A->get_filtered_tcp_ft_data());
    data_log->set_data_getContactedForceTorque(robot_A->get_contacted_ft_data());
    data_log->set_data_getPidCompensation(robot_A->get_pid_compensation());
    data_log->set_data_getDesiredTCPPose(robot_A->get_desired_pose_vector());
    data_log->set_data_new_line();

    //check if there is out of the real-time control
    previous_t = (rt_timer_read() - tstart)/1000000.0;
    if(previous_t > control_time*1000)
      cout << COLOR_RED_BOLD << "Exceed control time A "<< previous_t << COLOR_RESET << endl;

    rt_task_wait_period(NULL);
  }
}
void initialize()
{
  control_time = 0.002;
  std::string path_ = "/home/yik/sdu_ws/SDU-Control-Tasks/belt_task/config/initial_condition.yaml";

  robot_A = std::make_shared<UrRobot>(control_time);
  robot_A ->load_initialize_parameter(path_);

  //load motion data
  path_ = "/home/yik/sdu_ws/SDU-Control-Tasks/belt_task/config/motion/initialize_belt_task.yaml";
  robot_A ->load_task_motion(path_, "initialize_belt_task");
  path_ = "/home/yik/sdu_ws/SDU-Control-Tasks/belt_task/config/motion/initialize.yaml";
  robot_A ->load_task_motion(path_, "initialize");
  path_ = "/home/yik/sdu_ws/SDU-Control-Tasks/belt_task/config/motion/tcp_belt_task.yaml";
  robot_A ->load_task_motion(path_, "tcp_belt_task");

  //simulation check
  gazebo_check = true;
  //data log
  data_log = std::make_shared<DataLogging>();
  data_log->initialize();
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
  std::cin >> silmulation_on_off;

  if(!silmulation_on_off.compare("yes") || !silmulation_on_off.compare("y"))
    std::cout << COLOR_GREEN_BOLD << "Setting up Simulation " << COLOR_RESET << std::endl;
  else
  {
    std::cout << COLOR_GREEN_BOLD << "REAL Robot, Be careful to run:" << COLOR_RESET << std::endl;
    std::cout << COLOR_GREEN_BOLD << "Are you sure ? [yes / no]" << COLOR_RESET << std::endl;
    std::cin >> silmulation_on_off;
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

  if(gazebo_check)
  {
    ros_state->send_gazebo_command(robot_A->get_desired_q_());
  }
  else
  {
    robot_A->initialize("192.168.1.130");
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
    robot_A->robot_servo_stop();
  }
  return 0;
}
