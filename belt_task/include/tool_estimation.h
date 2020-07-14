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
#include <rw/math.hpp>

#include <Eigen/Dense>

#include "sdu_math/statistics_math.h"

using namespace rw::math;

class KalmanFilter
{
 public:
  KalmanFilter();
  ~KalmanFilter();
  void initialize_system(Eigen::Matrix<double, 6 ,6> F_init, Eigen::Matrix<double, 6 ,6> H_init, Eigen::Matrix<double, 6 ,6> Q_init, Eigen::Matrix<double, 6 ,6> R_init,
                         Eigen::Matrix<double, 6 ,6> B_init, Eigen::Matrix<double, 6 ,1> U_init, Eigen::Matrix<double, 6 ,1> Z_init);
  void change_noise_value(Eigen::Matrix<double, 6 ,6> R_init);
  void set_addtional_estimated_y_term(Eigen::Matrix<double, 6 ,1> add_term);
  void set_system_input_u(Eigen::Matrix<double, 6 ,1> input_u);
  Eigen::Matrix<double, 6 ,1> get_estimated_state();
  Eigen::Matrix<double, 6 ,1> get_output_error();
  Eigen::Matrix<double, 6 ,1> get_measurement_output_error();

  Eigen::Matrix<double, 6 ,6> get_kalman_gain_k();

  // kalman filter process
  void process_kalman_filtered_data(Eigen::Matrix<double, 6 ,1> measurement_y);

 private:
  // must be designed by your system model
  Eigen::Matrix<double, 6 ,6> F_;
  Eigen::Matrix<double, 6 ,6> H_;

  Eigen::Matrix<double, 6 ,6> Q_;
  Eigen::Matrix<double, 6 ,6> R_;

  Eigen::Matrix<double, 6 ,6> B_;
  Eigen::Matrix<double, 6 ,1> U_;
  Eigen::Matrix<double, 6 ,1> Z_;

  // intial condition
  Eigen::Matrix<double, 6 ,1> correction_value_x_;
  Eigen::Matrix<double, 6 ,6> correction_value_p_;

  // variables
  Eigen::Matrix<double, 6 ,1> prediction_value_x_;
  Eigen::Matrix<double, 6 ,6> prediction_value_p_;

  Eigen::Matrix<double, 6 ,1> previous_correction_value_x_;
  Eigen::Matrix<double, 6 ,6> previous_correction_value_p_;

  // kalman gain
  Eigen::Matrix<double, 6 ,6> kalman_gain_k_;

  // output variables
  Eigen::Matrix<double, 6 ,1> estimated_y_;
  Eigen::Matrix<double, 6 ,1> additonal_estimated_y_;
  Eigen::Matrix<double, 6 ,1> output_error_;
  Eigen::Matrix<double, 6 ,1> measurement_output_error_;
};

class LowPassFilter
{
 public:
  LowPassFilter();
  LowPassFilter(double control_time_init, double cutoff_frequency_init);
  ~LowPassFilter();
  void initialize();
  void set_parameters(double control_time_init, double cutoff_frequency_init, Eigen::Matrix<double, 6 ,1> data);
  Eigen::Matrix<double, 6 ,1> get_lpf_filtered_data(Eigen::Matrix<double, 6 ,1> data);  // frq = frequency , ctrl = contro
  double get_lpf_filtered_data(double data);

 private:
  double control_time_;
  double cutoff_frequency_;
  double raw_data_;
  double lambda_;
  double alpha_;

  Eigen::Matrix<double, 6 ,1> filtered_data_;
  Eigen::Matrix<double, 6 ,1> pre_filtered_data_;
};

class ToolEstimation
{
public:
  ToolEstimation();
  ~ToolEstimation();
  void initialize();

  void set_parameters(double control_time_init, double mass_of_tool_init);
  void set_noise_cov_parameters(double q_noise, double r_noise);
  void set_orientation_data(Transform3D<>  tf_base_to_tool);
  void set_gravity_input_data(std::vector<double> gravity_input);
  void set_sensor_offset_value(std::vector<double> raw_sensor_value);

  void process_estimated_force(std::vector<double> ft_data, std::vector<double> linear_acc_data);

  std::vector<double> get_contacted_force();
  std::vector<double> get_no_offset_contacted_force();
  std::vector<double> get_sensor_offset_value();

private:
  double control_time_;
  double mass_of_tool_;
  double cutoff_frequency_;

  //filter
  std::shared_ptr<KalmanFilter> kf_estimated_force;
  std::shared_ptr<LowPassFilter> lpf_force;

  //noise variables
  double r_,q_;

  Eigen::Matrix<double, 3, 3> orientation_base_to_tool_;
  Eigen::Matrix<double, 3, 1> gravity_;
  Eigen::Matrix<double, 3, 1> compensated_acc_;

  //force observer sensor
  Eigen::Matrix<double, 6 ,6> f_F_init_;
  Eigen::Matrix<double, 6 ,6> f_H_init_;
  Eigen::Matrix<double, 6 ,6> f_Q_init_;
  Eigen::Matrix<double, 6 ,6> f_R_init_;
  Eigen::Matrix<double, 6 ,6> f_B_init_;
  Eigen::Matrix<double, 6 ,1> f_U_init_;
  Eigen::Matrix<double, 6 ,1> f_Z_init_;

  Eigen::Matrix<double, 6 ,1> contacted_force_;
  Eigen::Matrix<double, 6 ,1> pre_contacted_force_;

  std::vector<double> get_no_offset_contacted_force_;
  std::vector<double> get_contacted_force_;
  std::vector<double> get_sensor_offset_;
};



#endif /* SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_TOOL_ESTIMATION_H_ */
