/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H
#define DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H


#include <ros/ros.h>

#include "dynamixel_workbench_controllers/message_header.h"

#include "sensor_msgs/JointState.h"
#include <t_flex/GoalPosition.h>

#include <t_flex/dynamixel_workbench.h>
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include "dynamixel_workbench_msgs/JointCommand.h"
#include <t_flex/JointSpeed.h>
#include <t_flex/TorqueEnable.h>


class PositionControl
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_state_list_pub_;
  ros::Publisher joint_states_pub_;

  // ROS Topic Subscriber
  ros::Subscriber joint_command_sub_;
  ros::Subscriber joint_position_sub_;

  // ROS Service Server
  ros::ServiceServer joint_command_server_;
  ros::ServiceServer joint_speed_server_;
  ros::ServiceServer joint_torque_enable_server_;
  // ROS Service Client

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;
  uint8_t dxl_id_[16];
  uint8_t dxl_cnt_;

public:
  PositionControl();
  ~PositionControl();
  void controlLoop(void);

 private:
  void initMsg();

  void initPublisher();
  void initSubscriber();
  void dynamixelStatePublish();
  void jointStatePublish();

  void initServer();
  bool jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                               dynamixel_workbench_msgs::JointCommand::Response &res);
  bool jointSpeedCallback(t_flex::JointSpeed::Request &req,
                          t_flex::JointSpeed::Response &res);
  bool jointTorqueEnableCallback(t_flex::TorqueEnable::Request &req,
                         	 t_flex::TorqueEnable::Response &res);
  void goalJointPositionCallback(const t_flex::GoalPosition::ConstPtr& msg);
};

#endif //DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H
