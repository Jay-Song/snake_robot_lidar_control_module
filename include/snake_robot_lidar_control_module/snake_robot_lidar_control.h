/*
 * snake_robot_lidar_control.h
 *
 *  Created on: Mar 3, 2017
 *      Author: jaysong
 */

#ifndef SNAKE_ROBOT_CONTROL_SNAKE_ROBOT_LIDAR_CONTROL_INCLUDE_SNAKE_ROBOT_LIDAR_CONTROL_SNAKE_ROBOT_LIDAR_CONTROL_H_
#define SNAKE_ROBOT_CONTROL_SNAKE_ROBOT_LIDAR_CONTROL_INCLUDE_SNAKE_ROBOT_LIDAR_CONTROL_SNAKE_ROBOT_LIDAR_CONTROL_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>

#include "robotis_framework_common/motion_module.h"


namespace heroehs
{

class SnakeRobotLidarControl : public robotis_framework::MotionModule, public robotis_framework::Singleton<SnakeRobotLidarControl>
{
public:
  SnakeRobotLidarControl();
  virtual ~SnakeRobotLidarControl();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void onModuleEnable();
  void onModuleDisable();

  void stop();
  bool isRunning();

  bool direction_change_;
  int  current_direction_;
  double rotating_angle_rad_;
  double goal_angle_rad_;
  double curr_angle_rad_;
  double value_to_rad_ratio_;

private:
  void queueThread();
  void publishDirectionChanging(bool is_changed);

  int    control_cycle_msec_;
  double control_cycle_sec_;

  boost::thread queue_thread_;

  ros::Publisher  dir_changing_pub_;
};

}

#endif /* SNAKE_ROBOT_CONTROL_SNAKE_ROBOT_LIDAR_CONTROL_INCLUDE_SNAKE_ROBOT_LIDAR_CONTROL_SNAKE_ROBOT_LIDAR_CONTROL_H_ */
