/*
 * main.cpp
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */
#include "belt_task.h"

void loop_robot_a_proc(void *arg)
{
  robot_path = robot_path + "/wc/UR10e_2018/UR10e.xml";
  robot_a->initialize(robot_path, "UR10e", robot_a_ip, gazebo_check);
  robot_a->move_to_init_pose();

  RT_TASK *curtask;
  RT_TASK_INFO curtaskinfo;

  curtask = rt_task_self();
  rt_task_inquire(curtask, &curtaskinfo);

  printf("Starting task %s with period of %f ms ....\n", curtaskinfo.name, control_time*1000);
  //Make the task periodic with a specified loop period

  RTIME tstart_A;
  rt_task_set_periodic(NULL, TM_NOW, LOOP_PERIOD);
  tstart_A = rt_timer_read();

  if(gazebo_check)
  {
    std::cout << COLOR_GREEN_BOLD << "Gazebo robot A start " << COLOR_RESET << std::endl;
  }

  bool task_completed = false;
  double task_time_A = 0.0;

  while(1)
  {
    ros_state->update_ros_data();
    tstart_A = rt_timer_read();

    robot_a->tasks(ros_state->get_task_command());
    robot_a->hybrid_controller();

    if(gazebo_check)
    {
      ros_state->send_gazebo_command(robot_a->get_current_q_());
    }

    ros_state->send_raw_ft_data(robot_a->get_raw_ft_data_());
    ros_state->send_filtered_ft_data(robot_a->get_contacted_ft_data_());
    ros_state->send_error_ee_pose(robot_a->get_error_ee_pose_());
    ros_state->send_ee_velocity(robot_a->get_actual_tcp_speed_());


    task_time_A = (rt_timer_read() - tstart_A)/1000000.0;
    cout << COLOR_GREEN_BOLD << "Check task's completion A : " << task_completed << " Elapsed time : "<< task_time_A << COLOR_RESET << endl;
    rt_task_wait_period(NULL);
  }
}
void loop_robot_b_proc(void *arg)
{
  robot_path = robot_path + "/wc/UR10e_2018/UR10e.xml";
  robot_b->initialize(robot_path, "UR10e", robot_b_ip, gazebo_check);

  robot_b->move_to_init_pose();

  RT_TASK *curtask;
  RT_TASK_INFO curtaskinfo;

  curtask = rt_task_self();
  rt_task_inquire(curtask, &curtaskinfo);

  printf("Starting task %s with period of %f ms ....\n", curtaskinfo.name, control_time*1000);
  //Make the task periodic with a specified loop period

  RTIME tstart_B;
  rt_task_set_periodic(NULL, TM_NOW, LOOP_PERIOD);
  tstart_B = rt_timer_read();

  if(gazebo_check)
  {
    std::cout << COLOR_GREEN_BOLD << "Gazebo robot B start " << COLOR_RESET << std::endl;
  }

  bool task_completed = false;
  double task_time_B = 0.0;

  while(1)
  {
    ros_state->update_ros_data();
    tstart_B = rt_timer_read();

    robot_b->tasks(ros_state->get_task_command());
    robot_b->hybrid_controller();

    if(gazebo_check)
    {
      ros_state->send_gazebo_command(robot_b->get_current_q_());
    }

    ros_state->send_raw_ft_data(robot_b->get_raw_ft_data_());
    ros_state->send_filtered_ft_data(robot_b->get_contacted_ft_data_());
    ros_state->send_error_ee_pose(robot_b->get_error_ee_pose_());
    ros_state->send_ee_velocity(robot_b->get_actual_tcp_speed_());

    task_time_B = (rt_timer_read() - tstart_B)/1000000.0;
    cout << COLOR_GREEN_BOLD << "Check task's completion B : " << task_completed << "  Elapsed time : "<< task_time_B << COLOR_RESET << endl;
    rt_task_wait_period(NULL);
  }
}
void initialize()
{
  robot_path = "/home/yik/sdu_ws/SDU-Control-Tasks/belt_task/config";
  robot_a = std::make_shared<TaskRobot>("robot_A",robot_path);
  robot_b = std::make_shared<TaskRobot>("robot_B",robot_path);
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

      robot_a->terminate_data_log();
      robot_b->terminate_data_log();

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
    ros_state->send_gazebo_command(robot_a->get_current_q_());
    //ros_state->send_gazebo_b_command(current_q);
  }

  mlockall(MCL_CURRENT | MCL_FUTURE); //Lock the memory to avoid memory swapping for this program

  signal(SIGINT, my_function);

  CPU_SET(3, &cpu_robot_a);    // select CPU 3
  CPU_SET(1, &cpu_robot_b);    // select CPU 1

  //real time task
  char str[35];
  sprintf(str, "Belt Task A Start");
  rt_task_create(&loop_robot_a, str, 0, 49, 0);//Create the real time task
  rt_task_set_affinity(&loop_robot_a, &cpu_robot_a);
  usleep(3000000);
  rt_task_start(&loop_robot_a, &loop_robot_a_proc, 0);//Since task starts in suspended mode, start task

  sprintf(str, "Belt Task B Start");
  rt_task_create(&loop_robot_b, str, 0, 50, 0);//Create the real time task
  rt_task_set_affinity(&loop_robot_b, &cpu_robot_b);
  usleep(3000000);
  rt_task_start(&loop_robot_b, &loop_robot_b_proc, 0);//Since task starts in suspended mode, start task

  std::cout << COLOR_GREEN << "Real time task loop was created!" << COLOR_RESET << std::endl;

  while(!exit_program)
  {
  }

  rt_task_delete(&loop_robot_a);
  rt_task_delete(&loop_robot_b);

  usleep(3000000);
  ros_state->shout_down_ros();
  usleep(3000000);

  // terminate robot
  if(!gazebo_check)
  {
    robot_a->terminate_robot();
    robot_b->terminate_robot();
  }
  robot_a->terminate_data_log();
  robot_b->terminate_data_log();
  return 0;
}
