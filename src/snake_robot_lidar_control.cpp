/*
 * snake_robot_lidar_control.cpp
 *
 *  Created on: Mar 3, 2017
 *      Author: jaysong
 */

#include "snake_robot_lidar_control_module/snake_robot_lidar_control.h"

using namespace heroehs;

SnakeRobotLidarControl::SnakeRobotLidarControl()
{
  enable_          = false;
  module_name_     = "lidar_control_module";
  control_mode_    = robotis_framework::PositionControl;

  result_["lidar_joint"] = new robotis_framework::DynamixelState();

  direction_change_ = false;
  current_direction_ = 1;
  rotating_angle_rad_ = 70.0*M_PI/180.0;
  goal_angle_rad_ = current_direction_ * rotating_angle_rad_;
  curr_angle_rad_ = 0;;
  value_to_rad_ratio_ = M_PI/4095.0;

  control_cycle_msec_ = 8;
  control_cycle_sec_ = control_cycle_msec_ * 0.001;
}

SnakeRobotLidarControl::~SnakeRobotLidarControl()
{
  queue_thread_.join();
}

void SnakeRobotLidarControl::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  queue_thread_ = boost::thread(boost::bind(&SnakeRobotLidarControl::queueThread, this));

  control_cycle_msec_ = control_cycle_msec;
  control_cycle_sec_ = control_cycle_msec_ * 0.001;

  value_to_rad_ratio_ = M_PI/4095.0;//robot->dxls_["lidar_joint"]->convertValue2Radian(1);
  goal_angle_rad_ = current_direction_ * (rotating_angle_rad_ + value_to_rad_ratio_ * 10.0);
}

void SnakeRobotLidarControl::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  dir_changing_pub_ = ros_node.advertise<std_msgs::Bool>("/heroehs/snake_robot/lidar_control/direction_changing", 5);

  while(ros_node.ok())
  {
    callback_queue.callAvailable();
    usleep(100);
  }
}

void SnakeRobotLidarControl::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
{
  if(enable_ == false)
    return;

  curr_angle_rad_ = dxls["lidar_joint"]->dxl_state_->present_position_;

  //ROS_INFO_STREAM("goal_angle_rad_ : " << goal_angle_rad_ << " curr_angle_rad_ : " << curr_angle_rad_);

  if(fabs(goal_angle_rad_ - curr_angle_rad_) < value_to_rad_ratio_ * 10.0)
  {
    current_direction_ = -1*current_direction_;
    goal_angle_rad_ = current_direction_ * (rotating_angle_rad_ + value_to_rad_ratio_ * 10.0);
    result_["lidar_joint"]->goal_position_ = goal_angle_rad_;

    publishDirectionChanging(true);
  }
}

void SnakeRobotLidarControl::onModuleEnable()
{
  current_direction_ = -1*current_direction_;
  goal_angle_rad_ = current_direction_ * (rotating_angle_rad_ + value_to_rad_ratio_ * 10.0);
  result_["lidar_joint"]->goal_position_ = goal_angle_rad_;
}

void SnakeRobotLidarControl::onModuleDisable()
{

}

void SnakeRobotLidarControl::stop()
{

}

bool SnakeRobotLidarControl::isRunning()
{
  return false;
}

void SnakeRobotLidarControl::publishDirectionChanging(bool is_changed)
{
  std_msgs::Bool msg;
  msg.data = is_changed;
  dir_changing_pub_.publish(msg);
}
