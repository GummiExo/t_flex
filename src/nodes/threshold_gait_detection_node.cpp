#include <stdint.h>
#include "agora/feature_extractor.hpp"
#include "agora/gait_cycle_classifier.hpp"
#include "ros/ros.h"
#include <t_flex/IMUData.h>
#include <t_flex/GaitPhase.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include "std_msgs/Bool.h"

using namespace std;

int32_t time_stamp;
double accel_y, gyro_y;
bool kill_flag = false;
string node_name = "threshold_gait_detection_node";

//instantiate the classifier
Gait_cycle_classifier *classifer = new Gait_cycle_classifier();
//used to receive the information of newly-recognized states
State_recognized_info latest_state_info;

void callback(const t_flex::IMUData imu_data){
  accel_y = imu_data.accel_y;
  gyro_y = imu_data.gyro_y;
  time_stamp = imu_data.time_stamp;

  //print received data out
  // cout << "time_stamp: " << time_stamp << ", accel_y: " << accel_y << ", gyro_y" << gyro_y << endl;

  //if a new state(phase) was recognized, print out the state and the time it was recognized at
  // Is the amplification neccesary?
  if (classifer->intake_data(400*accel_y, -100*gyro_y, time_stamp, latest_state_info)){
  // The gyroscope data needs to be inverted as the IMU from the original
  // package is oriented differently
  // if (classifer->intake_data(accel_y, -gyro_y, time_stamp, latest_state_info)){
  	cout << "Gait phase: " << get_state_string(latest_state_info.recognized_state) << " at time: " << latest_state_info.time_recognized << endl;
  }
}

void should_kill(const std_msgs::Bool::ConstPtr& kill_signal){
  kill_flag = kill_signal->data;
}

int main(int argc, char** argv){
  cout << "Initializing the classifier..." << endl;

  // ROS init
  ros::init(argc, argv, node_name);
  // NodeHandle is the main access point to communications with the ROS system.
	ros::NodeHandle n;

  // ROS publisher. Topic: gait_phase_detection
  ros::Publisher chatter_pub = n.advertise<t_flex::GaitPhase>("gait_phase_detection", 1000);
  // ROS subscriber. Topic: imu_data
  ros::Subscriber sub_imu = n.subscribe("imu_data", 1000, callback);
  // ROS subscriber. Topic: imu_data
  ros::Subscriber sub_kill = n.subscribe("kill_gait_assistance", 1000, should_kill);

  // Running at 500Hz
	ros::Rate loop_rate(500);
	ros::spinOnce();

  while (ros::ok()) {
    if (!kill_flag){
      // Publish GaitPhase message
      t_flex::GaitPhase gait_phase;
      gait_phase.header.frame_id = "/" + node_name;
      if (get_state_string(latest_state_info.recognized_state) == "Heel strike"){
        gait_phase.phase = 1;
      }
      else if (get_state_string(latest_state_info.recognized_state) == "Flat foot"){
        gait_phase.phase = 2;
      }
      else if (get_state_string(latest_state_info.recognized_state) == "Mid-stance"){
        gait_phase.phase = 3;
      }
      else if (get_state_string(latest_state_info.recognized_state) == "Heel-off"){
        gait_phase.phase = 4;
      }
      else if (get_state_string(latest_state_info.recognized_state) == "Toe-off"){
        gait_phase.phase = 5;
      }
      else if (get_state_string(latest_state_info.recognized_state) == "Mid-swing"){
        gait_phase.phase = 6;
      }
      // Invalid State!!
      else gait_phase.phase = 7;
      // cout << "Publishing gait_phases topic..." << endl;
      chatter_pub.publish(gait_phase);

      ros::spinOnce();
      loop_rate.sleep();
    }
    else break;
  }
	return 0;
}
