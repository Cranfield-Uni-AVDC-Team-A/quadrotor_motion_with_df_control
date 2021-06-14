/*!********************************************************************************
 * \brief     take_off behavior implementation 
 * \authors   Pablo Santamaria
 *            Miguel Fernandez Cortizas
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

#include "../include/behavior_take_off_with_df.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorTakeOffWithDF behavior;
  behavior.start();
  return 0;
}

BehaviorTakeOffWithDF::BehaviorTakeOffWithDF() : BehaviorExecutionManager(){ 
  setName("take_off_with_df");
  setExecutionGoal(ExecutionGoals::ACHIEVE_GOAL);
}

BehaviorTakeOffWithDF::~BehaviorTakeOffWithDF(){}

void BehaviorTakeOffWithDF::onConfigure(){
  nh = getNodeHandle();
  nspace = getNamespace();

  ros::param::get("~battery_topic", battery_topic);

  ros_utils_lib::getPrivateParam<std::string>("~estimated_pose_topic" 	    	          , estimated_pose_topic 			              ,"self_localization/pose");
  ros_utils_lib::getPrivateParam<std::string>("~flight_action_topic"		  	            , flight_action_topic    	              	,"actuator_command/flight_action");
  ros_utils_lib::getPrivateParam<std::string>("~status_topic"					                  , status_topic 					                  ,"self_localization/flight_state");
  ros_utils_lib::getPrivateParam<std::string>("~motion_reference_waypoints_path_topic"	, motion_reference_waypoints_path_topic   ,"motion_reference/waypoints");
  ros_utils_lib::getPrivateParam<std::string>("~battery_topic"	      , battery_topic          ,"sensor_measurement/battery_state");

  pose_sub_ = nh.subscribe("/" + nspace + "/" + estimated_pose_topic ,1,&BehaviorTakeOffWithDF::poseCallback,this);
  battery_subscriber = nh.subscribe("/" + nspace + "/"+battery_topic, 1, &BehaviorTakeOffWithDF::batteryCallback, this);
  status_sub = nh.subscribe("/" + nspace + "/"+status_topic, 1, &BehaviorTakeOffWithDF::statusCallBack, this);

  waypoints_references_pub_ = nh.advertise<aerostack_msgs::TrajectoryWaypoints>("/" + nspace + "/" + motion_reference_waypoints_path_topic, 1);
  flight_state_pub = nh.advertise<aerostack_msgs::FlightState>("/" + nspace + "/" + status_topic, 1);
  flight_action_pub = nh.advertise<aerostack_msgs::FlightActionCommand>("/" + nspace + "/" + flight_action_topic, 1);
  isLow=false;
}

void BehaviorTakeOffWithDF::onActivate(){
  std::string arguments = getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  if(config_file["speed"].IsDefined()){
    take_off_speed = config_file["speed"].as<double>(); 
  }
  if(config_file["altitude"].IsDefined()){
    take_off_altitude = config_file["altitude"].as<double>(); 
  }

  aerostack_msgs::FlightActionCommand msg;
  msg.header.stamp = ros::Time::now();
  msg.action = aerostack_msgs::FlightActionCommand::TAKE_OFF;
  flight_action_pub.publish(msg);

  activationPosition = position_;
  t_activacion_ = ros::Time::now();
  
}

void BehaviorTakeOffWithDF::onDeactivate(){
  aerostack_msgs::FlightActionCommand msg;
  msg.header.stamp = ros::Time::now();
  msg.action = aerostack_msgs::FlightActionCommand::HOVER;
  flight_action_pub.publish(msg);
}

void BehaviorTakeOffWithDF::onExecute(){
  static bool doOnce = true;
  if(doOnce && (ros::Time::now()-t_activacion_).toSec()>1){
    sendAltitudeSpeedReferences();
    doOnce = false;
  }
}

bool BehaviorTakeOffWithDF::checkSituation(){
  behavior_execution_manager_msgs::CheckSituation::Response rsp;
  if (status_msg.state != aerostack_msgs::FlightState::LANDED && status_msg.state != aerostack_msgs::FlightState::UNKNOWN){
    setErrorMessage("Error: Already flying");
    rsp.situation_occurs = false;
  }else if(isLow){
    setErrorMessage("Error: Battery low, unable to perform action");
    rsp.situation_occurs = false;
  }
  else{
    rsp.situation_occurs = true;
  }
  return rsp.situation_occurs;
}

void BehaviorTakeOffWithDF::batteryCallback(const sensor_msgs::BatteryState& battery) {
	if(battery.percentage * 100 < BATTERY_LOW_THRESHOLD) {
		isLow=true;
	}else{
    isLow=false;
  }
}

void BehaviorTakeOffWithDF::checkGoal(){
  // Check achievement
	if (checkTakeoff()){
    aerostack_msgs::FlightState msg;
    msg.header.stamp = ros::Time::now();
    msg.state = aerostack_msgs::FlightState::FLYING;
    flight_state_pub.publish(msg);
    BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
	}
}

void BehaviorTakeOffWithDF::checkProgress(){}

void BehaviorTakeOffWithDF::checkProcesses(){}

bool BehaviorTakeOffWithDF::checkTakeoff(){
	if (position_.z > activationPosition.z+take_off_altitude-0.05)
		return true;
	return false;
}

void BehaviorTakeOffWithDF::sendAltitudeSpeedReferences(){
  aerostack_msgs::TrajectoryWaypoints reference_waypoints;
  geometry_msgs::PoseStamped path_point;
  path_point.header.frame_id="odom";
  path_point.pose.position.x = (float)activationPosition.x;
  path_point.pose.position.y = (float)activationPosition.y;
  path_point.pose.position.z = (float)activationPosition.z+(float)take_off_altitude;
  reference_waypoints.poses.emplace_back(path_point);
  reference_waypoints.header.frame_id="odom";
  reference_waypoints.header.stamp = ros::Time::now();
  reference_waypoints.yaw_mode = YAW_MODE;
  reference_waypoints.max_speed = take_off_speed;
  waypoints_references_pub_.publish(reference_waypoints);
}

void BehaviorTakeOffWithDF::poseCallback(const geometry_msgs::PoseStamped& _msg){
	position_=_msg.pose.position;
}

void BehaviorTakeOffWithDF::statusCallBack(const aerostack_msgs::FlightState &msg){
  status_msg = msg;
}