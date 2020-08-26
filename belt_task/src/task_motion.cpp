/*
 * task_motion.cpp
 *
 *  Created on: Jun 15, 2020
 *      Author: yik
 */
#include "task_motion.h"


TaskMotion::TaskMotion()
{
  path_angle_ = 0;
  path_y_= 0;
  change_path_x_ = 0.001;
  change_path_y_ = 0.001;
  change_path_z_ = 0.001;
  phases_= 0;
  pre_phases_ = 0;

}
TaskMotion::~TaskMotion()
{

}

void TaskMotion::initialize(double control_time_, std::string load_path_)
{
  robot_traj = std::make_shared<EndEffectorTraj>();
  robot_traj->set_control_time(control_time_);

  current_point = -1; // wanna count from 0
  all_point = -1;
  tcp_all_point = -1;
  check_change = false;
  task_done = false;
  base_frame_ = true;

  desired_pose_matrix.resize(6,8);
  desired_pose_matrix.fill(0);

  // initial pose load
  desired_pose_matrix(0,7) = 5;
  desired_pose_matrix(1,7) = 5;
  desired_pose_matrix(2,7) = 5;
  desired_pose_matrix(3,7) = 5;
  desired_pose_matrix(4,7) = 5;
  desired_pose_matrix(5,7) = 5;

  current_pose_vector.resize(6);
  current_force_torque_vector.resize(6);

  YAML::Node doc; //
  try
  {
    // load yaml
    doc = YAML::LoadFile(load_path_.c_str());
  }catch(const std::exception& e)
  {
    std::cout << COLOR_RED_BOLD << "Fail to load data, yaml file!" << COLOR_RESET << std::endl;
    return;
  }

  // motion data load initialize//
  YAML::Node bigger_pulley_bearing_position_node = doc["bigger_pulley_bearing_position"];
  YAML::Node task_initial_position_node = doc["task_initial_position"];

  for(int num = 0; num < 6; num ++)
  {
    bigger_pulley_bearing_position.push_back(bigger_pulley_bearing_position_node[num].as<double>());
    smaller_pulley_bearing_position.push_back(0);
  }

  for(int num = 0; num < 3; num ++)
    task_initial_position.push_back(task_initial_position_node[num].as<double>());
  for(int num = 3; num < 6; num ++)
    task_initial_position.push_back(task_initial_position_node[num].as<double>()*DEGREE2RADIAN);


  tf_base_to_bearing = Transform3D<> (Vector3D<>(bigger_pulley_bearing_position[0], bigger_pulley_bearing_position[1], bigger_pulley_bearing_position[2]), EAA<>(bigger_pulley_bearing_position[3], bigger_pulley_bearing_position[4], bigger_pulley_bearing_position[5]).toRotation3D());
  tf_bearing_to_init = Transform3D<> (Vector3D<>(task_initial_position[0], task_initial_position[1], task_initial_position[2]), RPY<>(task_initial_position[3],task_initial_position[4], task_initial_position[5]).toRotation3D());

  tf_base_to_init_task = tf_base_to_bearing*tf_bearing_to_init;

  for(int num = 0; num < 3; num ++)
    initial_robot_ee_position.push_back(tf_base_to_init_task.P()[num]);
  for(int num = 0; num < 3; num ++)
    initial_robot_ee_position.push_back(EAA<>(tf_base_to_init_task.R())[num]);

  set_initial_pose(initial_robot_ee_position[0], initial_robot_ee_position[1], initial_robot_ee_position[2], initial_robot_ee_position[3], initial_robot_ee_position[4], initial_robot_ee_position[5]); // set to be robot initial values

  std::cout << "tf_base_to_init_task : " << tf_base_to_init_task << std::endl;
  std::cout << "initial_robot_ee_position : "<<initial_robot_ee_position << std::endl;
}
void TaskMotion::robot_initialize() // joint space
{

}
void TaskMotion::load_task_motion(std::string path_, std::string motion_)
{
  YAML::Node doc; //
  try
  {
    // load yaml
    doc = YAML::LoadFile(path_.c_str());
  }catch(const std::exception& e)
  {
    std::cout << COLOR_RED_BOLD << "Fail to load data, yaml file!" << COLOR_RESET << std::endl;
    return;
  }

  // motion data load initialize//
  YAML::Node motion_start_time_node = doc["motion_start_time"];
  //YAML::Node motion_task_node = doc["motion_task"];

  std::vector<double> temp_motion_start_time_vector;
  std::vector<double> temp_motion_task_pose_vector;
  std::vector<double> temp_motion_task_vel_vector;

  int point_numbers;
  point_numbers = 0;

  for (YAML::iterator it = motion_start_time_node.begin(); it != motion_start_time_node.end(); ++it)
  {
    point_numbers = it->first.as<int>();
    temp_motion_start_time_vector.push_back(it->second[0].as<double>());
    motion_start_time_vector[point_numbers] = temp_motion_start_time_vector;
    temp_motion_start_time_vector.clear();

    for(int num = 0; num < 3; num++)
    {
      temp_motion_task_pose_vector.push_back(initial_robot_ee_position[num]);
      temp_motion_task_vel_vector.push_back(0);
    }
    for(int num = 3; num < 6; num++)
    {

      temp_motion_task_pose_vector.push_back(initial_robot_ee_position[num]);
      temp_motion_task_vel_vector.push_back(0);

    }
    motion_task_pose_vector[point_numbers] = temp_motion_task_pose_vector;
    all_point ++;
  }

  for(int num = 0; num < all_point+1 ; num++)
  {
    motion_task_init_vel_vector[num] = temp_motion_task_vel_vector;
    motion_task_final_vel_vector[num] = temp_motion_task_vel_vector;
  }

  temp_motion_start_time_vector.clear();
  temp_motion_task_pose_vector.clear();
  temp_motion_task_vel_vector.clear();

  init_all_point = all_point;
  init_motion_start_time_vector = motion_start_time_vector;
  init_motion_task_pose_vector = motion_task_pose_vector;
  init_motion_task_init_vel_vector = motion_task_init_vel_vector;
  init_motion_task_final_vel_vector = motion_task_final_vel_vector;
  all_point = -1;

  base_frame_ = true;
  std::cout << "LOAD initial pose Complete" << std::endl;
}
void TaskMotion::trans_tcp_to_base_motion(std::string load_path_)
{
  YAML::Node doc; //
  try
  {
    // load yaml
    doc = YAML::LoadFile(load_path_.c_str());
  }catch(const std::exception& e)
  {
    std::cout << COLOR_RED_BOLD << "Fail to load data, yaml file!" << COLOR_RESET << std::endl;
    return;
  }

  // motion data load initialize//
  YAML::Node motion_start_time_node = doc["motion_start_time"];
  YAML::Node motion_task_node = doc["motion_task"];
  YAML::Node force_node = doc["desired_force"];

  std::vector<double> temp_motion_start_time_vector;
  std::vector<double> temp_motion_task_pose_vector;
  std::vector<double> temp_motion_task_vel_vector;
  std::vector<double> temp_force_node_vector;

  int point_numbers;
  point_numbers = 0;

  //time
  for (YAML::iterator it = motion_start_time_node.begin(); it != motion_start_time_node.end(); ++it)
  {
    point_numbers = it->first.as<int>();
    temp_motion_start_time_vector.push_back(it->second[0].as<double>());
    tcp_motion_start_time_vector[point_numbers] = temp_motion_start_time_vector;
    temp_motion_start_time_vector.clear();
  }
  //points
  for (YAML::iterator it = motion_task_node.begin(); it != motion_task_node.end(); ++it)
  {
    point_numbers = it->first.as<int>();
    for(int num = 0; num < 3; num++)
    {
      temp_motion_task_pose_vector.push_back(it->second[num].as<double>());
    }
    for(int num = 3; num < 6; num++)
    {
      temp_motion_task_pose_vector.push_back(it->second[num].as<double>());
    }
    tcp_motion_task_pose_vector[point_numbers] = temp_motion_task_pose_vector;
    temp_motion_task_pose_vector.clear();

    tcp_all_point ++;
  }
  //force
  for (YAML::iterator it = force_node.begin(); it != force_node.end(); ++it)
  {
    point_numbers = it->first.as<int>();
    for(int num = 0; num < 6; num++)
    {
      temp_force_node_vector.push_back(it->second[num].as<double>());
    }
    tcp_motion_desired_force_vector[point_numbers] = temp_force_node_vector;
    temp_force_node_vector.clear();
  }
  std::cout << "TCP task motion LOAD and Transformation (TCP --> base) Complete" << std::endl;
}
void TaskMotion::change_motion(std::string motion_)
{
  if(!motion_.compare("initialize"))
  {
    base_frame_ = true;

    all_point = init_all_point;

    motion_start_time_vector = init_motion_start_time_vector;
    motion_task_pose_vector  = init_motion_task_pose_vector;
    motion_task_init_vel_vector = init_motion_task_init_vel_vector;
    motion_task_final_vel_vector = init_motion_task_final_vel_vector;

    std::cout << "receive initialize command" << std::endl;
  }
  else if(!motion_.compare("tcp_belt_task"))
  {
    all_point = tcp_all_point;

    static std::vector<double> temp_motion_task_vel_vector;
    static RPY<> rpy;
    motion_task_pose_vector = tcp_motion_task_pose_vector;
    motion_start_time_vector = tcp_motion_start_time_vector;

    for(int num = 0; num < all_point+1; num ++)
    {
      rpy = RPY<>(tcp_motion_task_pose_vector[num][3]*DEGREE2RADIAN,tcp_motion_task_pose_vector[num][4]*DEGREE2RADIAN,tcp_motion_task_pose_vector[num][5]*DEGREE2RADIAN);

      tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(tcp_motion_task_pose_vector[num][0], tcp_motion_task_pose_vector[num][1], tcp_motion_task_pose_vector[num][2]),
          rpy.toRotation3D());

      tf_desired_pose_ = tf_current_pose_* tf_tcp_desired_pose_;

      motion_task_pose_vector[num][0] = Vector3D<> (tf_desired_pose_.P())[0];
      motion_task_pose_vector[num][1] = Vector3D<> (tf_desired_pose_.P())[1];
      motion_task_pose_vector[num][2] = Vector3D<> (tf_desired_pose_.P())[2];

      motion_task_pose_vector[num][3] = EAA<> (tf_desired_pose_.R())[0];
      motion_task_pose_vector[num][4] = EAA<> (tf_desired_pose_.R())[1];
      motion_task_pose_vector[num][5] = EAA<> (tf_desired_pose_.R())[2];
    }

    //velocity
    for(int num = 0; num < 6; num++)
    {
      temp_motion_task_vel_vector.push_back(0);
    }

    for(int num = 0; num < all_point+1 ; num++)
    {
      motion_task_init_vel_vector[num] = temp_motion_task_vel_vector;
      motion_task_final_vel_vector[num] = temp_motion_task_vel_vector;
    }
    temp_motion_task_vel_vector.clear();

    base_frame_ = false;

    std::cout << "receive tcp_belt_task command" << std::endl;
  }
  else
    std::cout << "invalid command" << std::endl;
}
bool TaskMotion::make_belt_robust(double radious)
{
  static RPY<> tcp_rpy_;
  static RPY<> pulley_frame_rpy_;
  static Vector3D<> pulley_frame_xyz_;

  if(phases_ == 0)
  {
    tcp_rpy_ = RPY<>(0,0,0);

    tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(radious, 0, 0), tcp_rpy_.toRotation3D());

    tf_desired_pose_ = tf_base_to_init_task*tf_tcp_desired_pose_;

    desired_pose_matrix(0,1) = Vector3D<> (tf_desired_pose_.P())[0];
    desired_pose_matrix(1,1) = Vector3D<> (tf_desired_pose_.P())[1];
    desired_pose_matrix(2,1) = Vector3D<> (tf_desired_pose_.P())[2];

    desired_pose_matrix(3,1) = EAA<> (tf_desired_pose_.R())[0];
    desired_pose_matrix(4,1) = EAA<> (tf_desired_pose_.R())[1];
    desired_pose_matrix(5,1) = EAA<> (tf_desired_pose_.R())[2];

    for(int num = 0; num <6; num ++)
    {
      desired_pose_matrix(num,7) = 5;
    }
  }
}
bool TaskMotion::close_to_pulleys(double x,double y,double depth)
{
  static RPY<> tcp_rpy_;
  static RPY<> pulley_frame_rpy_;
  static Vector3D<> pulley_frame_xyz_;

  if(pre_phases_ != phases_)
    tf_static_frame_ = tf_current_pose_;

  // output always has to be points in relative to base frame (global)
  if(phases_ == 1)
  {
    tcp_rpy_ = RPY<>(0,0,0);

    tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(x, y, depth), tcp_rpy_.toRotation3D());

    tf_desired_pose_ = tf_static_frame_*tf_tcp_desired_pose_;

    desired_pose_matrix(0,1) = Vector3D<> (tf_desired_pose_.P())[0];
    desired_pose_matrix(1,1) = Vector3D<> (tf_desired_pose_.P())[1];
    desired_pose_matrix(2,1) = Vector3D<> (tf_desired_pose_.P())[2];

    desired_pose_matrix(3,1) = EAA<> (tf_desired_pose_.R())[0];
    desired_pose_matrix(4,1) = EAA<> (tf_desired_pose_.R())[1];
    desired_pose_matrix(5,1) = EAA<> (tf_desired_pose_.R())[2];

    for(int num = 0; num <6 ; num ++)
    {
      desired_pose_matrix(num,7) = 2.5;
    }
  }
}
bool TaskMotion::insert_belt_into_pulley(bool contact_, double change_x, double change_y, double change_z)
{
  static RPY<> tcp_rpy_;
  static RPY<> pulley_frame_rpy_;
  static Vector3D<> pulley_frame_xyz_;
  //
  if(pre_phases_ != phases_)
    tf_static_frame_ = tf_current_pose_;

  // output always has to be points in relative to base frame (global)
  if(phases_ == 2)
  {
    tcp_rpy_ = RPY<>(0,0,0);

    tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(change_x, change_y, change_z), tcp_rpy_.toRotation3D());

    tf_desired_pose_ = tf_static_frame_*tf_tcp_desired_pose_;

    desired_pose_matrix(0,1) = Vector3D<> (tf_desired_pose_.P())[0];
    desired_pose_matrix(1,1) = Vector3D<> (tf_desired_pose_.P())[1];
    desired_pose_matrix(2,1) = Vector3D<> (tf_desired_pose_.P())[2];

    desired_pose_matrix(3,1) = EAA<> (tf_desired_pose_.R())[0];
    desired_pose_matrix(4,1) = EAA<> (tf_desired_pose_.R())[1];
    desired_pose_matrix(5,1) = EAA<> (tf_desired_pose_.R())[2];

    for(int num = 0; num <6 ; num ++)
    {
      desired_pose_matrix(num,7) = 7;
    }
  }
}
bool TaskMotion::up_motion(bool contact_, double x, double y, double z, double axis_x, double axis_y, double axis_z)
{
  static RPY<> tcp_rpy_;
  static RPY<> pulley_frame_rpy_;
  static Vector3D<> pulley_frame_xyz_;
  //
  if(pre_phases_ != phases_)
    tf_static_frame_ = tf_current_pose_;

  // output always has to be points in relative to base frame (global)
  if(phases_ == 3)
  {
    tcp_rpy_ = RPY<>(axis_x,axis_y,axis_z);

    tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(x, y, z), tcp_rpy_.toRotation3D());

    tf_desired_pose_ = tf_static_frame_*tf_tcp_desired_pose_;

    desired_pose_matrix(0,1) = Vector3D<> (tf_desired_pose_.P())[0];
    desired_pose_matrix(1,1) = Vector3D<> (tf_desired_pose_.P())[1];
    desired_pose_matrix(2,1) = Vector3D<> (tf_desired_pose_.P())[2];

    desired_pose_matrix(3,1) = EAA<> (tf_desired_pose_.R())[0];
    desired_pose_matrix(4,1) = EAA<> (tf_desired_pose_.R())[1];
    desired_pose_matrix(5,1) = EAA<> (tf_desired_pose_.R())[2];

    for(int num = 0; num <6 ; num ++)
    {
      desired_pose_matrix(num,7) = 2;
    }
  }
}
bool TaskMotion::finish_1(bool contact_, double x, double y, double z, double axis_x, double axis_y, double axis_z)
{
  static RPY<> tcp_rpy_;
  static RPY<> pulley_frame_rpy_;
  static Vector3D<> pulley_frame_xyz_;
  //
  if(pre_phases_ != phases_)
    tf_static_frame_ = tf_current_pose_;

  // output always has to be points in relative to base frame (global)
  if(phases_ == 4)
  {
    tcp_rpy_ = RPY<>(axis_x,axis_y,axis_z);

    tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(x, y, z), tcp_rpy_.toRotation3D());

    tf_desired_pose_ = tf_static_frame_*tf_tcp_desired_pose_;

    desired_pose_matrix(0,1) = Vector3D<> (tf_desired_pose_.P())[0];
    desired_pose_matrix(1,1) = Vector3D<> (tf_desired_pose_.P())[1];
    desired_pose_matrix(2,1) = Vector3D<> (tf_desired_pose_.P())[2];

    desired_pose_matrix(3,1) = EAA<> (tf_desired_pose_.R())[0];
    desired_pose_matrix(4,1) = EAA<> (tf_desired_pose_.R())[1];
    desired_pose_matrix(5,1) = EAA<> (tf_desired_pose_.R())[2];

    for(int num = 0; num <6 ; num ++)
    {
      desired_pose_matrix(num,7) = 1;
    }
  }
}
bool TaskMotion::finish_2(bool contact_, double x, double y, double z, double axis_x, double axis_y, double axis_z)
{
  static RPY<> tcp_rpy_;
  static RPY<> pulley_frame_rpy_;
  static Vector3D<> pulley_frame_xyz_;
  //
  if(pre_phases_ != phases_)
    tf_static_frame_ = tf_current_pose_;

  // output always has to be points in relative to base frame (global)
  if(phases_ == 5)
  {
    tcp_rpy_ = RPY<>(axis_x,axis_y,axis_z);

    tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(x, y, z), tcp_rpy_.toRotation3D());

    tf_desired_pose_ = tf_static_frame_*tf_tcp_desired_pose_;

    desired_pose_matrix(0,1) = Vector3D<> (tf_desired_pose_.P())[0];
    desired_pose_matrix(1,1) = Vector3D<> (tf_desired_pose_.P())[1];
    desired_pose_matrix(2,1) = Vector3D<> (tf_desired_pose_.P())[2];

    desired_pose_matrix(3,1) = EAA<> (tf_desired_pose_.R())[0];
    desired_pose_matrix(4,1) = EAA<> (tf_desired_pose_.R())[1];
    desired_pose_matrix(5,1) = EAA<> (tf_desired_pose_.R())[2];

    for(int num = 0; num <6 ; num ++)
    {
      desired_pose_matrix(num,7) = 1;
    }
  }
}
void TaskMotion::check_phases()
{
  pre_phases_ = phases_;
  if(robot_traj->is_moving_check == false && phases_ < 6)
  {
    phases_ ++;
  }
}
void TaskMotion::run_task_motion()
{
  if(all_point == -1)
    return;

  if(robot_traj->is_moving_check !=true)// not during motion --> can recieve first or new point.
  {
    static RPY<> tcp_rpy;

    if(task_done)
      return;

    if(robot_traj->is_moving_check != check_change) // point change is detected.
    {
      current_point ++;

      std::cout << current_point << std::endl;

      if(!base_frame_ && current_point <= all_point)
      {
        motion_task_pose_vector = tcp_motion_task_pose_vector;

        tcp_rpy = RPY<>(tcp_motion_task_pose_vector[current_point][3]*DEGREE2RADIAN,tcp_motion_task_pose_vector[current_point][4]*DEGREE2RADIAN,tcp_motion_task_pose_vector[current_point][5]*DEGREE2RADIAN);

        tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(tcp_motion_task_pose_vector[current_point][0], tcp_motion_task_pose_vector[current_point][1], tcp_motion_task_pose_vector[current_point][2]),
            tcp_rpy.toRotation3D());

        tf_desired_pose_ = tf_current_pose_* tf_tcp_desired_pose_;

        motion_task_pose_vector[current_point][0] = Vector3D<> (tf_desired_pose_.P())[0];
        motion_task_pose_vector[current_point][1] = Vector3D<> (tf_desired_pose_.P())[1];
        motion_task_pose_vector[current_point][2] = Vector3D<> (tf_desired_pose_.P())[2];

        motion_task_pose_vector[current_point][3] = EAA<> (tf_desired_pose_.R())[0];
        motion_task_pose_vector[current_point][4] = EAA<> (tf_desired_pose_.R())[1];
        motion_task_pose_vector[current_point][5] = EAA<> (tf_desired_pose_.R())[2];
      }
    }
    else
      current_point = 0; //or stop the robot

    if(current_point > all_point)
    {
      check_change = robot_traj->is_moving_check;
      std::cout << "Finished motion!" << std::endl;
      task_done = true;
      return;
    }

    if(current_point == -1)
      return;

    calculate_init_final_velocity(current_point);

    for(int num = 0; num <6; num ++)
    {
      desired_pose_matrix(num,1) = motion_task_pose_vector[current_point][num];

      desired_pose_matrix(num,2) = motion_task_init_vel_vector[0][num];
      desired_pose_matrix(num,3) = motion_task_final_vel_vector[current_point][num];

      if(!base_frame_)
      {
        desired_pose_matrix(num,2) = 0;
        desired_pose_matrix(num,3) = 0;
      }

      desired_pose_matrix(num,7) = motion_start_time_vector[current_point][0];
    }
    // change the point
  }
  else // during motion
  {
  }

  if(!base_frame_)
  {
    tf_tcp_desired_force_ = Wrench6D<> (tcp_motion_desired_force_vector[current_point][0], tcp_motion_desired_force_vector[current_point][1], tcp_motion_desired_force_vector[current_point][2],
        tcp_motion_desired_force_vector[current_point][3], tcp_motion_desired_force_vector[current_point][4], tcp_motion_desired_force_vector[current_point][5]);

    tf_force_desired_ = tf_current_pose_.R()*tf_tcp_desired_force_;

    for(int num = 0; num <3; num ++)
    {
      current_force_torque_vector[num] = tf_force_desired_.force()[num];
    }
  }
  check_change = robot_traj->is_moving_check;
}
void TaskMotion::generate_trajectory()
{
  robot_traj->cal_end_point_to_rad(desired_pose_matrix);

  for(int num = 0; num <6 ; num ++)
  {
    current_pose_vector[num] = robot_traj->get_traj_results()(num,0);
  }
}
void TaskMotion::set_initial_pose(double x, double y, double z, double axis_x, double axis_y, double axis_z)
{
  current_pose_vector[0] = x;
  current_pose_vector[1] = y;
  current_pose_vector[2] = z;
  current_pose_vector[3] = axis_x;
  current_pose_vector[4] = axis_y;
  current_pose_vector[5] = axis_z;

  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix(num,0) =  current_pose_vector[num];
    desired_pose_matrix(num,1) =  current_pose_vector[num];
    robot_traj->current_pose_change(num,0) = current_pose_vector[num];
  }

  robot_traj->cal_end_point_tra_px->current_pose = current_pose_vector[0];
  robot_traj->cal_end_point_tra_py->current_pose = current_pose_vector[1];
  robot_traj->cal_end_point_tra_pz->current_pose = current_pose_vector[2];
  robot_traj->cal_end_point_tra_alpha->current_pose = current_pose_vector[3];
  robot_traj->cal_end_point_tra_betta->current_pose = current_pose_vector[4];
  robot_traj->cal_end_point_tra_kamma->current_pose = current_pose_vector[5];
}
void TaskMotion::set_point(double x, double y, double z, double axis_x, double axis_y, double axis_z)
{
  current_pose_vector[0] = x;
  current_pose_vector[1] = y;
  current_pose_vector[2] = z;
  current_pose_vector[3] = axis_x;
  current_pose_vector[4] = axis_y;
  current_pose_vector[5] = axis_z;

  clear_task_motion();
  std::cout << "receive set point command" << std::endl;
}
void TaskMotion::set_current_pose_eaa(double x, double y, double z, double axis_x, double axis_y, double axis_z)
{
  tf_current_pose_ = Transform3D<> (Vector3D<>(x, y, z), EAA<>(axis_x, axis_y, axis_z).toRotation3D());
}
void TaskMotion::clear_task_motion()
{
  task_done = false;
  all_point = -1;
  current_point = -1;
  motion_start_time_vector.clear();
  motion_task_pose_vector.clear();

  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix(num,0) =  current_pose_vector[num];
    desired_pose_matrix(num,1) =  current_pose_vector[num];
    robot_traj->current_pose_change(num,0) = current_pose_vector[num];
  }
  robot_traj->cal_end_point_tra_px->current_pose = current_pose_vector[0];
  robot_traj->cal_end_point_tra_py->current_pose = current_pose_vector[1];
  robot_traj->cal_end_point_tra_pz->current_pose = current_pose_vector[2];
  robot_traj->cal_end_point_tra_alpha->current_pose = current_pose_vector[3];
  robot_traj->cal_end_point_tra_betta->current_pose = current_pose_vector[4];
  robot_traj->cal_end_point_tra_kamma->current_pose = current_pose_vector[5];
}
void TaskMotion::clear_phase()
{
  phases_ = 0;
}

double TaskMotion::calculate_velocity(double first_point,double second_point, double interval_time)
{
  return (second_point - first_point)/interval_time;
}
void TaskMotion::calculate_init_final_velocity(int point_number)
{
  static double first_vel = 0;
  static double second_vel = 0;

  if(point_number < 0)
    return;

  if(all_point < 1) // in case of one point
  {
    for(int var = 0; var <6 ; var++)
    {
      motion_task_init_vel_vector[0][var] = 0;
      motion_task_final_vel_vector[0][var] = 0;
    }
    return;
  }

  if(point_number == 0) // start phase
  {
    for(int var = 0; var <6 ; var++)
    {
      first_vel  = calculate_velocity(current_pose_vector[var],motion_task_pose_vector[0][var], motion_start_time_vector[0][0]);
      second_vel = calculate_velocity(motion_task_pose_vector[0][var],motion_task_pose_vector[1][var], motion_start_time_vector[1][0]);

      motion_task_init_vel_vector[0][var] = 0;
      motion_task_final_vel_vector[0][var] = calculate_next_velocity(first_vel, second_vel);
    }
    return;
  }
  if(point_number == all_point) // final phase
  {
    for(int var = 0; var <6 ; var++)
    {
      motion_task_init_vel_vector[point_number][var] = motion_task_final_vel_vector[point_number-1][var];
      motion_task_final_vel_vector[point_number][var] = 0;
    }
    return;
  }

  // medium phase
  for(int var = 0; var <6 ; var++)
  {
    first_vel  = calculate_velocity(motion_task_pose_vector[point_number-1][var],motion_task_pose_vector[point_number][var], motion_start_time_vector[point_number][0]); //
    second_vel = calculate_velocity(motion_task_pose_vector[point_number][var],motion_task_pose_vector[point_number+1][var], motion_start_time_vector[point_number+1][0]); //

    motion_task_init_vel_vector[point_number][var] = motion_task_final_vel_vector[point_number-1][var];
    motion_task_final_vel_vector[point_number][var] = calculate_next_velocity(first_vel, second_vel);
  }

}
double TaskMotion::calculate_next_velocity(double first_vel, double second_vel)
{
  if(first_vel*second_vel > 0)
    return second_vel;
  else
    return 0;
}
void TaskMotion::tf_set_point_base(std::vector<double> tcp_set_point)
{
  static RPY<> tcp_rpy_;
  set_point_.clear();

  tcp_rpy_ = RPY<>(tcp_set_point[3],tcp_set_point[4],tcp_set_point[5]);

  tf_tcp_desired_pose_ = Transform3D<> (Vector3D<>(tcp_set_point[0],tcp_set_point[1],tcp_set_point[2]),
      tcp_rpy_.toRotation3D());

  tf_desired_pose_ = tf_current_pose_* tf_tcp_desired_pose_;

  set_point_.push_back(Vector3D<> (tf_desired_pose_.P())[0]);
  set_point_.push_back(Vector3D<> (tf_desired_pose_.P())[1]);
  set_point_.push_back(Vector3D<> (tf_desired_pose_.P())[2]);
  set_point_.push_back(EAA<> (tf_desired_pose_.R())[0]);
  set_point_.push_back(EAA<> (tf_desired_pose_.R())[1]);
  set_point_.push_back(EAA<> (tf_desired_pose_.R())[2]);
}
void TaskMotion::stop_motion()
{
  desired_pose_matrix.fill(0);
  for(int num = 0; num <6 ; num ++)
  {
    desired_pose_matrix(num,0) = current_pose_vector[num];
    desired_pose_matrix(num,1) = current_pose_vector[num];
    desired_pose_matrix(num,7) = 1;
  }
  robot_traj->stop_trajectory();
}
void TaskMotion::set_pulley_radious(double radious)
{
  radious_ = radious;
}
std::vector<double> TaskMotion::get_current_pose()
{
  return current_pose_vector;
}
std::vector<double> TaskMotion::get_set_point_base()
{
  return set_point_;
}
std::vector<double> TaskMotion::get_desired_force_torque()
{
  return current_force_torque_vector;
}
std::vector<double> TaskMotion::get_initial_ee_position()
{
  return initial_robot_ee_position;
}
unsigned int TaskMotion::get_phases_()
{
  return phases_;
}

