/*
 * belt_task.cpp
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */
/*
 * tool_estimation.cpp
 *
 *  Created on: Feb 12, 2020
 *      Author: yik
 */
#include "tool_estimation.h"

ToolEstimation::ToolEstimation()
{
  initialize();
}

ToolEstimation::~ToolEstimation()
{
  delete kf_estimated_force;
}

void ToolEstimation::initialize()
{
  control_time_ = 0.002;
  mass_of_tool_ = 1.72;
  cutoff_frequency_ = 10;

  r_ = 1000;
  q_ = 0.1;

  kf_estimated_force = new KalmanFilter;

  contacted_force_.resize(6, 1);
  pre_contacted_force_.resize(6, 1);
  orientation_base_to_tool_.resize(3,3);
  orientation_base_to_tool_.fill(0);
  contacted_force_.fill(0);
  pre_contacted_force_.fill(0);

  //filtered_acc_
  gravity_.resize(3,1);
  gravity_.fill(0);

  compensated_acc_.resize(3,1);
  compensated_acc_.fill(0);

  // force contact model design initialize
  f_F_init_.resize(6, 6);
  f_H_init_.resize(6, 6);
  f_Q_init_.resize(6, 6);
  f_R_init_.resize(6, 6);
  f_B_init_.resize(6, 6);
  f_U_init_.resize(6, 1);
  f_Z_init_.resize(6, 1);

  f_F_init_.setIdentity();
  f_H_init_.setIdentity();

  f_B_init_.setZero();
  f_Q_init_.setIdentity();
  f_R_init_.setIdentity();
  f_Z_init_.setZero();
  f_U_init_.setZero();
  f_Q_init_ = f_Q_init_ * q_;
  f_R_init_ = f_R_init_ * r_;

  kf_estimated_force->initialize_system(f_F_init_, f_H_init_, f_Q_init_, f_R_init_, f_B_init_, f_U_init_, f_Z_init_);
}

void ToolEstimation::set_parameters(double control_time_init, double mass_of_tool_init)
{
  control_time_ = control_time_init;
  mass_of_tool_ = mass_of_tool_init;
}
void ToolEstimation::set_noise_cov_parameters(double q_noise, double r_noise)
{
  q_ = q_noise;
  r_ = r_noise;
}
void ToolEstimation::set_orientation_data(Eigen::MatrixXd tf_base_to_tool)
{
  orientation_base_to_tool_ = tf_base_to_tool.block(0,0,3,3);
}
void ToolEstimation::set_gravity_input_data(Eigen::MatrixXd gravity_input)
{
  gravity_ = orientation_base_to_tool_*gravity_input;
}
std::vector<double> ToolEstimation::get_estimated_force(Eigen::MatrixXd ft_data, Eigen::MatrixXd linear_acc_data)  // input entire force torque
{
  if(orientation_base_to_tool_.determinant() == 0) // inverse check
    return get_contacted_force_;

  get_contacted_force_.clear();

  compensated_acc_ = linear_acc_data - (orientation_base_to_tool_.inverse()*gravity_);

  compensated_acc_ = orientation_base_to_tool_ * compensated_acc_;

  ft_data(0,0) =  ft_data(0,0) - (mass_of_tool_*compensated_acc_)(0,0);
  ft_data(1,0) =  ft_data(1,0) - (mass_of_tool_*compensated_acc_)(1,0);
  ft_data(2,0) =  ft_data(2,0) - (mass_of_tool_*compensated_acc_)(2,0);

  kf_estimated_force->process_kalman_filtered_data(ft_data);

  contacted_force_ = kf_estimated_force->get_estimated_state();
  contacted_force_ = contacted_force_*0.01 + pre_contacted_force_*0.99;
  pre_contacted_force_ = contacted_force_;

  for(unsigned int num = 0 ; num < 6 ; num ++)
  {
    get_contacted_force_.push_back(contacted_force_(num,0));
  }
  return get_contacted_force_;
}


