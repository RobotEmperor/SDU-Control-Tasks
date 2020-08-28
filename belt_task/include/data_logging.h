/*
 * data_logging.h
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */

#ifndef SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_DATA_LOGGING_H_
#define SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_DATA_LOGGING_H_

#include <Eigen/Dense>
// file load
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>

//yaml
#include <yaml-cpp/yaml.h>

class DataLogging
{
public:
  DataLogging(std::string robot_name);
  ~DataLogging();

  void initialize();
  void save_file();
  void set_time_count(double sampling_time);
  void set_data_getActualQ(std::vector<double> acutal_q);
  void set_data_getActualTCPPose(std::vector<double> actual_pose);
  void set_data_getTargetTCPPose(std::vector<double> target_pose);
  void set_data_getActualTCPForceTorque(std::vector<double> acutal_ft);
  void set_data_getActualToolAccelerometer(std::vector<double> acutal_acc);
  void set_data_getFilteredTCPForceTorque(std::vector<double> filtered_tcp_ft);
  void set_data_getContactedForceTorque(std::vector<double> contacted_ft);
  void set_data_getPidCompensation(std::vector<double> pid_compensation);
  void set_data_getDesiredTCPPose(std::vector<double> desired_pose);
  void set_data_getBeltPosition(std::vector<double> belt_position);
  void set_data_getDesiredBeltPosition(std::vector<double> desired_belt_position);
  void set_data_new_line();
  std::string data_change_to_string(std::vector<double> data);

private:
  std::ofstream* out;
  std::string path_;
  std::string data_line_;
  std::string time_count_;
  std::string getActualQ_;
  std::string getActualTCPPose_;
  std::string getTargetTCPPose_;
  std::string getActualTCPForceTorque_;
  std::string getActualToolAccelerometer_;
  std::string getFilteredTCPForceTorque_;
  std::string getContactedForceTorque_;
  std::string getPidCompensation_;
  std::string getDesiredTCPPose_;
  std::string getBeltPosition_;
  std::string getDesiredBeltPosition_;


};


#endif /* SDU_CONTROL_TASKS_BELT_TASK_INCLUDE_DATA_LOGGING_H_ */
