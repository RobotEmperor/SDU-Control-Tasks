/*
 * belt_task.h
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */
#ifndef SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_BELT_TASK_H_
#define SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_BELT_TASK_H_
#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT
#define CLOCK_RES 1e-9 //Clock resolution is 1 us by default 1e-9
#define LOOP_PERIOD 2e6 //Expressed in ticks // 2ms control time

#include "log.h"
#include "ros_node.h"
#include "task_robot.h"
#include <signal.h>

//xenomai rt system
#include <unistd.h>
#include <signal.h>
#include <cstdlib>
#include <sys/mman.h>
#include <sys/types.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <mutex>

//real time task
RT_TASK loop_robot_a;
RT_TASK loop_robot_b;

//cpu set to avoid overlaod
cpu_set_t cpu_robot;

void initialize();

//robot load
std::shared_ptr<TaskRobot> robot_a;
std::shared_ptr<TaskRobot> robot_b;

std::string robot_a_ip;
std::string robot_b_ip;
std::string robot_path;

std::string silmulation_on_off;
//ros
std::shared_ptr<RosNode> ros_state;
double control_time;
bool gazebo_check;
bool exit_program;

std::mutex m;

#endif /* SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_BELT_TASK_H_ */
