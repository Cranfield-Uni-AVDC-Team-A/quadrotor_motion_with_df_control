/*!********************************************************************************
 * \brief     take_off behavior implementation 
 * \authors   Pablo Santamaria
 * \copyright Copyright (c) 2021 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#ifndef TAKE_OFF_WITH_DF_H
#define TAKE_OFF_WITH_DF_H

// ROS
#include "std_msgs/Float32MultiArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

// Aerostack msgs
#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>
#include <aerostack_msgs/FlightActionCommand.h>
#include <sensor_msgs/BatteryState.h>
#include <aerostack_msgs/FlightState.h>

// Aerostack libraries
#include <BehaviorExecutionManager.h>
#include "ros_utils_lib/ros_utils.hpp"
#include "ros_utils_lib/control_utils.hpp"

#define TAKEOFF_ALTITUDE 1.0f
#define TAKEOFF_SPEED 0.1f

class BehaviorTakeOffWithDF : public BehaviorExecutionManager
{
  // Constructor
public:
  BehaviorTakeOffWithDF();
  ~BehaviorTakeOffWithDF();
  int main(int argc, char** argv);

private:
  
  ros::NodeHandle nh;
  std::string nspace;

  // Config variables
  
  std::string battery_topic; 

	std::string estimated_pose_topic;
	std::string flight_action_topic;
	std::string status_topic;
	std::string motion_reference_waypoints_path_topic;

  const double BATTERY_LOW_THRESHOLD = 10;
  bool isLow;

  ros::Time t_activacion_;

  // Communication variables
  ros::Subscriber status_sub;
  ros::Subscriber battery_subscriber;

  ros::Publisher flight_action_pub;

  ros::Subscriber pose_sub_;
  ros::Subscriber speeds_sub_;
  ros::Subscriber flight_action_sub;
  ros::Publisher path_references_pub_;
  ros::Publisher flight_state_pub;

  // Messages
  aerostack_msgs::FlightState status_msg;

  geometry_msgs::Point position_;
  geometry_msgs::Point activationPosition;

private:
  // BehaviorExecutionManager
  void onConfigure();
  void onActivate();
  void onDeactivate();
  void onExecute();
  bool checkSituation();
  void checkGoal();
  void checkProgress();
  void checkProcesses();

  bool checkTakeoff();
  void sendAltitudeSpeedReferences(const double& dz_speed , const double takeoff_altitude = TAKEOFF_ALTITUDE);

public: // Callbacks
  void statusCallBack(const aerostack_msgs::FlightState &msg);
  void batteryCallback(const sensor_msgs::BatteryState& battery);
  void poseCallback(const geometry_msgs::PoseStamped&);
  void flightActionCallback(const aerostack_msgs::FlightActionCommand& _msg);
};

#endif
