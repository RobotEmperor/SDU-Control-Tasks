/*
 * data_logging.cpp
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */
#include "data_logging.h"


DataLogging::DataLogging(std::string path)
{
  path_ = path;
}
DataLogging::~DataLogging()
{
  delete out;
}
void DataLogging::initialize()
{
  out = new std::ofstream;
  out->open(path_+"/"+"data_log.csv");

  //initial position
  //data variables define
  getTargetTCPPose_ = " target_tcp_pose_x target_tcp_pose_y target_tcp_pose_z target_tcp_pose_eaa_x target_tcp_pose_eaa_y target_tcp_pose_eaa_z";
  getActualTCPPose_ = " actual_tcp_pose_x actual_tcp_pose_y actual_tcp_pose_z actual_tcp_pose_eaa_x actual_tcp_pose_eaa_y actual_tcp_pose_eaa_z";
  getDesiredTCPPose_ = " desired_tcp_pose_x desired_tcp_pose_y desired_tcp_pose_z desired_tcp_pose_eaa_x desired_tcp_pose_eaa_y desired_tcp_pose_eaa_z";
  getActualTCPForceTorque_ = " actual_force_x actual_force_y actual_force_z actual_torque_x actual_torque_x actual_torque_x";
  getFilteredTCPForceTorque_ = " filtered_tcp_force_x filtered_tcp_force_y filtered_tcp_force_z filtered_tcp_torque_x filtered_tcp_torque_y filtered_tcp_torque_z";
  getContactedForceTorque_ = " contacted_force_x contacted_force_y contacted_force_z contacted_torque_x contacted_torque_y contacted_torque_z";
  getActualToolAccelerometer_ = " actual_tcp_acc_x actual_tcp_acc_y actual_tcp_acc_z";
  getActualQ_ = " actual_q_0 actual_q_1 actual_q_2 actual_q_3 actual_q_4 actual_q_5";
  getPidCompensation_ = " x_compensation y_compensation z_compensation eaa_x_compensation eaa_y_compensation eaa_z_compensation";
  getBeltPosition_ = " belt_x belt_y belt_z";
  getDesiredBeltPosition_ = " desired_belt_x desired_belt_y desired_belt_z";///////////////////////////////

  data_line_ = "time"+getTargetTCPPose_+getActualTCPPose_+getActualTCPForceTorque_+getFilteredTCPForceTorque_+getContactedForceTorque_+
      getActualToolAccelerometer_+getActualQ_+getPidCompensation_+getDesiredTCPPose_+getBeltPosition_+getDesiredBeltPosition_+"\n";//////////////////////////////////

  //out << data_line << std::endl;
  out->write(data_line_.c_str(),data_line_.size());

  time_count_ = "";
  getTargetTCPPose_ = "";
  getActualTCPPose_ = "";
  getActualTCPForceTorque_ = "";
  getFilteredTCPForceTorque_ = "";
  getContactedForceTorque_ = "";
  getActualToolAccelerometer_ = "";
  getActualQ_ = "";
  getPidCompensation_ = "";
  getDesiredTCPPose_ = "";
  getBeltPosition_ = "";
  getDesiredBeltPosition_ = "";////////////////////////////////

}
void DataLogging::save_file()
{
  out->close();
  std::cout << "complete and save" << std::endl;
}
//data recording
void DataLogging::set_time_count(double sampling_time)
{
  time_count_ = std::to_string(sampling_time);
}
void DataLogging::set_data_getActualQ(std::vector<double> acutal_q)
{
  getActualQ_ = data_change_to_string(acutal_q);
}
void DataLogging::set_data_getActualTCPPose(std::vector<double> actual_pose)
{
  getActualTCPPose_ = data_change_to_string(actual_pose);
}
void DataLogging::set_data_getDesiredTCPPose(std::vector<double> desired_pose)
{
  getDesiredTCPPose_= data_change_to_string(desired_pose);
}
void DataLogging::set_data_getTargetTCPPose(std::vector<double> target_pose)
{
  getTargetTCPPose_ = data_change_to_string(target_pose);
}
void DataLogging::set_data_getActualTCPForceTorque(std::vector<double> acutal_ft)
{
  getActualTCPForceTorque_ = data_change_to_string(acutal_ft);
}
void DataLogging::set_data_getFilteredTCPForceTorque(std::vector<double> filtered_tcp_ft)
{
  getFilteredTCPForceTorque_ = data_change_to_string(filtered_tcp_ft);
}
void DataLogging::set_data_getContactedForceTorque(std::vector<double> contacted_ft)
{
  getContactedForceTorque_ = data_change_to_string(contacted_ft);
}
void DataLogging::set_data_getPidCompensation(std::vector<double> pid_compensation)
{
  getPidCompensation_ = data_change_to_string(pid_compensation);
}
void DataLogging::set_data_getBeltPosition(std::vector<double> belt_position)
{
  if(belt_position.size() == 0)
    return;
  for(int num = 0; num < 3; num++)
  {
    getBeltPosition_ += " "+std::to_string(belt_position[num]);
  }
}
void DataLogging::set_data_getDesiredBeltPosition(std::vector<double> desired_belt_position)///////////////////////////////
{
  if(desired_belt_position.size() == 0)
    return;
  for(int num = 0; num < 3; num++)
  {
    getDesiredBeltPosition_ += " "+std::to_string(desired_belt_position[num]);
  }
}
void DataLogging::set_data_getActualToolAccelerometer(std::vector<double> acutal_acc)
{
  if(acutal_acc.size() == 0)
    return;
  for(int num = 0; num < 3; num++)
  {
    getActualToolAccelerometer_ += " "+std::to_string(acutal_acc[num]);
  }
}
void DataLogging::set_data_new_line()
{
  data_line_ = time_count_+getTargetTCPPose_+getActualTCPPose_+getActualTCPForceTorque_+getFilteredTCPForceTorque_+getContactedForceTorque_+getActualToolAccelerometer_+getActualQ_+getPidCompensation_+getDesiredTCPPose_+getBeltPosition_+getDesiredBeltPosition_+"\n";////////////////////

  out->write(data_line_.c_str(),data_line_.size());

  time_count_ = "";
  getTargetTCPPose_ = "";
  getActualTCPPose_ = "";
  getActualTCPForceTorque_ = "";
  getFilteredTCPForceTorque_ = "";
  getContactedForceTorque_ = "";
  getActualToolAccelerometer_ = "";
  getActualQ_ = "";
  getPidCompensation_ = "";
  getDesiredTCPPose_="";
  getBeltPosition_ = "";
  getDesiredBeltPosition_ = "";/////////////////////////
}
std::string DataLogging::data_change_to_string(std::vector<double> data)
{
  std::string temp_string;

  if(data.size() == 0)
    return "";

  for(int num = 0; num<6; num++)
  {
    temp_string +=" "+std::to_string(data[num]);
  }
  return temp_string;
}

