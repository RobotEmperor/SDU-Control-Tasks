/*
 * tool_estimation.h
 *
 *  Created on: Jun 16, 2020
 *      Author: yik
 */

#ifndef SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_TOOL_ESTIMATION_H_
#define SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_TOOL_ESTIMATION_H_


#include <math.h>
#include <stdio.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include "sdu_math/statistics_math.h"
#include "sensor_filter/sensor_filter.h"


class ToolEstimation
{
public:
  ToolEstimation();
  ~ToolEstimation();
  void initialize();

  void set_parameters(double control_time_init, double mass_of_tool_init);
  void set_noise_cov_parameters(double q_noise, double r_noise);
  void set_orientation_data(Eigen::MatrixXd tf_base_to_tool);
  void set_gravity_input_data(Eigen::MatrixXd gravity_input);

  std::vector<double> get_estimated_force(Eigen::MatrixXd ft_data, Eigen::MatrixXd linear_acc_data);

private:
  double control_time_;
  double mass_of_tool_;
  double cutoff_frequency_;

  //filter
  KalmanFilter* kf_estimated_force;

  //noise variables
  double r_,q_;

  Eigen::MatrixXd orientation_base_to_tool_;
  Eigen::MatrixXd gravity_;
  Eigen::MatrixXd compensated_acc_;

  //force observer sensor
  Eigen::MatrixXd f_F_init_;
  Eigen::MatrixXd f_H_init_;
  Eigen::MatrixXd f_Q_init_;
  Eigen::MatrixXd f_R_init_;
  Eigen::MatrixXd f_B_init_;
  Eigen::MatrixXd f_U_init_;
  Eigen::MatrixXd f_Z_init_;

  Eigen::MatrixXd contacted_force_;
  Eigen::MatrixXd pre_contacted_force_;

  std::vector<double> get_contacted_force_;
};



#endif /* SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_TOOL_ESTIMATION_H_ */
