/*
 * belt_task.h
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */
#ifndef SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_BELT_TASK_H_
#define SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_BELT_TASK_H_

#include <stdio.h>
#include <signal.h>
#include <Eigen/Dense>

#include "ur_robot.h"

//xenomai rt system
#include <unistd.h>
#include <signal.h>
#include <cstdlib>
#include <sys/mman.h>
#include <sys/types.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>

#include "ros_node.h"
#include "data_logging.h"

#define CLOCK_RES 1e-9 //Clock resolution is 1 us by default 1e-9
#define LOOP_PERIOD 2e6 //Expressed in ticks // 2ms control time
RT_TASK loop_task;

void initialize();

bool exit_program;

//model definition
std::string silmulation_on_off;
bool gazebo_check;

std::shared_ptr<RosNode> ros_state;
std::shared_ptr<DataLogging> data_log;

//robot select
std::shared_ptr<UrRobot> robot_A;

//control
double control_time;

#endif /* SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_BELT_TASK_H_ */
