#include <stdint.h>
#include "t_flex/feature_extractor.hpp"
#include "t_flex/gait_cycle_classifier.hpp"
#include "ros/ros.h"
#include "t_flex/IMUData.h"
#include "t_flex/GaitPhase.h"
#include <cstdlib>
#include <iostream>
#include <string>

using namespace std;

int32_t time_stamp;
double accel_y, gyro_y;
string node_name = "threshold_detection_node";

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
  /* Constants for original IMU class */
  // if (classifer->intake_data(4*accel_y, -6*gyro_y, time_stamp, latest_state_info)){
  /* Constants for IMU class with gyro/16.0 and accel/100.0 */
  //if (classifer->intake_data(400*accel_y, -100*gyro_y, time_stamp, latest_state_info)){
  if (classifer->intake_data(400*accel_y, 500*gyro_y, time_stamp, latest_state_info)){
  // The gyroscope data needs to be inverted as the IMU from the original
  // package is oriented differently
  // if (classifer->intake_data(accel_y, -gyro_y, time_stamp, latest_state_info)){
  	cout << "Gait phase: " << get_state_string(latest_state_info.recognized_state) << " at time: " << latest_state_info.time_recognized << endl;
  }
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
  ros::Subscriber sub = n.subscribe("imu_data/paretic", 1000, callback);

  // Running at 10Hz
	ros::Rate loop_rate(10);
	ros::spinOnce();

  while (ros::ok()) {
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
	return 0;
}
