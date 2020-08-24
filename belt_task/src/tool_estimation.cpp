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

KalmanFilter::KalmanFilter()
{
	//intial condition
	correction_value_x_.fill(0);
	correction_value_p_.setIdentity();

	//variables
	prediction_value_x_.fill(0);
	prediction_value_p_.setIdentity();

	previous_correction_value_x_.fill(0);
	previous_correction_value_p_.setIdentity();

	previous_correction_value_x_ = correction_value_x_;
	previous_correction_value_p_ = correction_value_p_;

	kalman_gain_k_.fill(0);
	estimated_y_.fill(0);
	additonal_estimated_y_.fill(0);
	output_error_.fill(0);
	measurement_output_error_.fill(0);
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::initialize_system (Eigen::Matrix<double, 6 ,6> F_init, Eigen::Matrix<double, 6 ,6> H_init, Eigen::Matrix<double, 6 ,6> Q_init, Eigen::Matrix<double, 6 ,6> R_init,
		Eigen::Matrix<double, 6 ,6> B_init, Eigen::Matrix<double, 6 ,1> U_init, Eigen::Matrix<double, 6 ,1> Z_init)
{
	//must be designed by your system model
	F_ = F_init;
	H_ = H_init;
	Q_ = Q_init;
	R_ = R_init;
	B_ = B_init;
	U_ = U_init;
	Z_ = Z_init;
}

void KalmanFilter::process_kalman_filtered_data(Eigen::Matrix<double, 6, 1> measurement_y)
{
	prediction_value_x_ = F_ * previous_correction_value_x_ + B_ * U_;

	prediction_value_p_ = F_ * correction_value_p_ * F_.transpose() + Q_;

	if ((H_ * prediction_value_p_ * H_.transpose() + R_).determinant() == 0)
	{
		return;
	}

	kalman_gain_k_ = prediction_value_p_ * H_.transpose() * ((H_ * prediction_value_p_ * H_.transpose() + R_).inverse());

	estimated_y_ = H_ * prediction_value_x_ + additonal_estimated_y_;

	correction_value_x_ = prediction_value_x_ + kalman_gain_k_ * (measurement_y - estimated_y_);

	correction_value_p_ = prediction_value_p_ - (kalman_gain_k_ * H_ * prediction_value_p_);

	previous_correction_value_x_ = correction_value_x_;
	previous_correction_value_p_ = correction_value_p_;

	measurement_output_error_ = measurement_y - estimated_y_;
	output_error_(0,0) = measurement_y(0,0) - correction_value_x_(0,0);
}

void KalmanFilter::change_noise_value(Eigen::Matrix<double, 6, 6> R_init)
{
	R_ = R_init;
}
void KalmanFilter::set_addtional_estimated_y_term(Eigen::Matrix<double, 6, 1> add_term)
{
	additonal_estimated_y_ = add_term;
}
void KalmanFilter::set_system_input_u(Eigen::Matrix<double, 6, 1> input_u)
{
	U_ = input_u;
}
Eigen::Matrix<double, 6, 1> KalmanFilter::get_estimated_state()
{
	return correction_value_x_;
}
Eigen::Matrix<double, 6, 1> KalmanFilter::get_output_error()
{
	return output_error_;
}
Eigen::Matrix<double, 6, 1> KalmanFilter::get_measurement_output_error()
{
	return measurement_output_error_;
}
Eigen::Matrix<double, 6, 6> KalmanFilter::get_kalman_gain_k()
{
	return kalman_gain_k_;
}

LowPassFilter::LowPassFilter()
{
	control_time_ = 0;
	cutoff_frequency_ = 0;
	lambda_ = 0;
	alpha_ = 0;
	raw_data_ = 0;
}

LowPassFilter::LowPassFilter(double control_time_init, double cutoff_frequency_init)
: control_time_(control_time_init), cutoff_frequency_(cutoff_frequency_init)
{
	this->initialize();
}

LowPassFilter::~LowPassFilter()
{
}

void LowPassFilter::initialize()
{
	lambda_ = 0;
	alpha_ = 0;
	raw_data_ = 0;

	lambda_ = 1 / (2 * M_PI * cutoff_frequency_);
	alpha_ = control_time_ / (lambda_ + control_time_);
}

void LowPassFilter::set_parameters(double control_time_init, double cutoff_frequency_init, Eigen::Matrix<double, 6 ,1> data)
{
	control_time_ = control_time_init;
	cutoff_frequency_ = cutoff_frequency_init;

	filtered_data_.resize(data.rows(), data.cols());
	pre_filtered_data_.setZero(data.rows(), data.cols());
}

Eigen::Matrix<double, 6 ,1> LowPassFilter::get_lpf_filtered_data(Eigen::Matrix<double, 6 ,1> data)
{
	for (int data_num = 0; data_num < data.size(); data_num++)
	{
		filtered_data_(data_num, 0) = alpha_ * data(data_num, 0) + ((1 - alpha_) * pre_filtered_data_(data_num, 0));
		pre_filtered_data_(data_num, 0) = filtered_data_(data_num, 0);
	}
	return filtered_data_;
}
double LowPassFilter::get_lpf_filtered_data(double data)
{
	static double filtered_data_d_;
	static double pre_filtered_data_d_;

	filtered_data_d_= alpha_ * data + ((1 - alpha_) * pre_filtered_data_d_);
	pre_filtered_data_d_= filtered_data_d_;

	return filtered_data_d_;
}


ToolEstimation::ToolEstimation()
{
	initialize();
}
ToolEstimation::~ToolEstimation()
{
}
void ToolEstimation::initialize()
{
	control_time_ = 0.002;
	mass_of_tool_ = 0;
	cutoff_frequency_ = 3;

	r_ = 20;
	q_ = 0.1;

	kf_estimated_force = std::make_shared<KalmanFilter>();
	lpf_force = std::make_shared<LowPassFilter>();

	//filtered_acc_
	gravity_.fill(0);
	compensated_acc_.fill(0);

	orientation_base_to_tool_.fill(0);
	contacted_force_.fill(0);
	pre_contacted_force_.fill(0);

	// force contact model design initialize
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
	lpf_force->set_parameters(control_time_, 3, contacted_force_);
	lpf_force->initialize();
	get_sensor_offset_.assign(6,0);
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
void ToolEstimation::set_orientation_data(Transform3D<> tf_base_to_tool)
{
	static Eigen::Matrix<double, 3,3> tf_base_to_tool_m_;

	for(unsigned int num_row = 0; num_row < 3; num_row ++)
	{
		for(unsigned int num_col = 0; num_col < 3; num_col ++)
		{
			tf_base_to_tool_m_(num_row,num_col) = tf_base_to_tool(num_row,num_col);
		}
	}
	orientation_base_to_tool_ = tf_base_to_tool_m_.block(0,0,3,3);
}
void ToolEstimation::set_gravity_input_data(std::vector<double> gravity_input)
{
	static Eigen::Matrix<double, 3,1> gravity_input_m_;

	for(unsigned int num_row = 0; num_row < gravity_input.size(); num_row ++)
	{
		gravity_input_m_(num_row,0) = gravity_input[num_row];
	}

	gravity_ = orientation_base_to_tool_*gravity_input_m_;
}
void ToolEstimation::set_sensor_offset_value(std::vector<double> raw_sensor_value)
{
	for(unsigned int num = 0 ; num < raw_sensor_value.size() ; num ++)
	{
		get_sensor_offset_[num] = raw_sensor_value[num];
	}
}
void ToolEstimation::process_estimated_force(std::vector<double> ft_data, std::vector<double> linear_acc_data)  // input entire force torque
{
	static Eigen::Matrix<double, 6, 1>  ft_data_m_;
	static Eigen::Matrix<double, 3, 1>  linear_acc_data_m_;

	get_contacted_force_.clear();

	for(unsigned int num_row = 0; num_row < ft_data.size(); num_row ++)
	{
		ft_data_m_(num_row,0) = ft_data[num_row];
	}
	for(unsigned int num_row = 0; num_row < linear_acc_data.size(); num_row ++)
	{
		linear_acc_data_m_(num_row,0) = linear_acc_data[num_row];
	}

	if(orientation_base_to_tool_.determinant() == 0) // inverse check
		return;

	compensated_acc_ = linear_acc_data_m_ - (orientation_base_to_tool_.inverse()*gravity_);

	compensated_acc_ = orientation_base_to_tool_ * compensated_acc_;

	ft_data_m_(0,0) =  ft_data_m_(0,0) + (mass_of_tool_*compensated_acc_)(0,0);
	ft_data_m_(1,0) =  ft_data_m_(1,0) + (mass_of_tool_*compensated_acc_)(1,0);
	ft_data_m_(2,0) =  ft_data_m_(2,0) + (mass_of_tool_*compensated_acc_)(2,0);
	ft_data_m_(3,0) =  ft_data_m_(3,0);
	ft_data_m_(4,0) =  ft_data_m_(4,0);
	ft_data_m_(5,0) =  ft_data_m_(5,0);

	kf_estimated_force->process_kalman_filtered_data(ft_data_m_);

	contacted_force_ = kf_estimated_force->get_estimated_state();

	//contacted_force_ = lpf_force->get_lpf_filtered_data(contacted_force_);

	for(unsigned int num = 0 ; num < 6 ; num ++)
	{
		get_contacted_force_.push_back(contacted_force_(num,0) - get_sensor_offset_[num]);
	}

	for(unsigned int num = 0 ; num < 6 ; num ++)
	{
		get_no_offset_contacted_force_.push_back(contacted_force_(num,0));
	}
}
std::vector<double> ToolEstimation::get_contacted_force()
{
	return get_contacted_force_;
}
std::vector<double> ToolEstimation::get_sensor_offset_value()
{
	return get_sensor_offset_;
}
std::vector<double> ToolEstimation::get_no_offset_contacted_force()
{
	return get_no_offset_contacted_force_;
}


