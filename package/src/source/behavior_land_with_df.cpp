/*!********************************************************************************
 * \brief     land behavior implementation 
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

#include "../include/behavior_land_with_df.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorLandWithDF behavior;
  behavior.start();
  return 0;
}

BehaviorLandWithDF::BehaviorLandWithDF() : BehaviorExecutionManager(){ 
  setName("land_with_df");
  setExecutionGoal(ExecutionGoals::ACHIEVE_GOAL);
}

BehaviorLandWithDF::~BehaviorLandWithDF(){}

void BehaviorLandWithDF::onConfigure(){ 
  nh = getNodeHandle();
  nspace = getNamespace();

  ros_utils_lib::getPrivateParam<std::string>("~estimated_pose_topic" 	    	          , estimated_pose_topic 			              ,"self_localization/pose");
  ros_utils_lib::getPrivateParam<std::string>("~flight_action_topic"		  	            , flight_action_topic    	              	,"actuator_command/flight_action");
  ros_utils_lib::getPrivateParam<std::string>("~status_topic"					                  , status_topic 					                  ,"self_localization/flight_state");
  ros_utils_lib::getPrivateParam<std::string>("~motion_reference_waypoints_path_topic"	, motion_reference_waypoints_path_topic   ,"motion_reference/waypoints");
  ros_utils_lib::getPrivateParam<std::string>("~actuator_command_thrust_topic"          , actuator_command_thrust_topic           ,"actuator_command/thrust");

  pose_sub_ = nh.subscribe("/" + nspace + "/" + estimated_pose_topic ,1,&BehaviorLandWithDF::poseCallback,this);
  status_sub = nh.subscribe("/" + nspace + "/"+status_topic, 1, &BehaviorLandWithDF::statusCallBack, this);
  thrust_sub = nh.subscribe("/" + nspace + "/" + actuator_command_thrust_topic, 1, &BehaviorLandWithDF::thrustCallBack, this);

  waypoints_references_pub_ = nh.advertise<aerostack_msgs::TrajectoryWaypoints>("/" + nspace + "/" + motion_reference_waypoints_path_topic, 1);
  flight_state_pub = nh.advertise<aerostack_msgs::FlightState>("/" + nspace + "/" + status_topic, 1);
  flight_action_pub = nh.advertise<aerostack_msgs::FlightActionCommand>("/" + nspace + "/" + flight_action_topic, 1);
}

void BehaviorLandWithDF::onActivate(){
  std::string arguments = getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  if(config_file["speed"].IsDefined()){
    land_speed = config_file["speed"].as<double>(); 
  }

  lastAltitude = ros::Time::now();
  aerostack_msgs::FlightActionCommand msg;
  msg.header.stamp = ros::Time::now();
  msg.action = aerostack_msgs::FlightActionCommand::LAND;
  flight_action_pub.publish(msg);

  activationPosition = position_;
  activationThrust = thrust_;
  t_activacion_ = ros::Time::now();
  sendAltitudeSpeedReferences(land_speed,land_altitude);
}

void BehaviorLandWithDF::onDeactivate(){
}

void BehaviorLandWithDF::onExecute(){
}

bool BehaviorLandWithDF::checkSituation(){
  behavior_execution_manager_msgs::CheckSituation::Response rsp;
  if (status_msg.state == aerostack_msgs::FlightState::LANDED){
    setErrorMessage("Error: Already landed");
    rsp.situation_occurs = false;
  }else if(status_msg.state == aerostack_msgs::FlightState::LANDING){
    setErrorMessage("Error: Already landing");
    rsp.situation_occurs = false;
  }
  else{
    rsp.situation_occurs = true;
  }
  return rsp.situation_occurs;
}

void BehaviorLandWithDF::checkGoal(){
  // Check achievement
	if (checkLanding()){
    aerostack_msgs::FlightState msg;
    msg.header.stamp = ros::Time::now();
    msg.state = aerostack_msgs::FlightState::LANDED;
    flight_state_pub.publish(msg);
    BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
	}
}

void BehaviorLandWithDF::checkProgress(){}

void BehaviorLandWithDF::checkProcesses(){}

bool BehaviorLandWithDF::checkLanding(){
  if((ros::Time::now() - lastAltitude).toSec() > 0.1){
    if (altitudes_list.size()==LAND_CONFIRMATION_SECONDS*10){
      altitudes_list.pop_front();
    }
    altitudes_list.push_back(position_.z);
    lastAltitude = ros::Time::now();
  }
	if (altitudes_list.size()==LAND_CONFIRMATION_SECONDS*10){
    double avg = 0;
    double var = 0;
    double squared_sum = 0;
    std::list<float>::iterator it;
    for(it = altitudes_list.begin(); it != altitudes_list.end(); it++){
      avg += *it;
      squared_sum += (*it)*(*it);
    }
    avg /= altitudes_list.size();
    var = (squared_sum/altitudes_list.size())-(avg*avg);
    /*if(sqrt(var)<land_speed && confirmed_movement){
      altitudes_list.empty();
      confirmed_movement = false;
	    return true;
    }
    else */if (sqrt(var)<land_speed && activationThrust/4>thrust_){
      altitudes_list.empty();
      confirmed_movement = false;
	    return true;
    }
    if(sqrt(var)>land_speed && (*altitudes_list.begin()-*altitudes_list.end())>land_speed){
      confirmed_movement = true;
    }
  }
  if (position_.z < land_altitude || abs(position_.z - land_altitude)*land_speed < LAND_CONFIRMATION_SECONDS*land_speed){
    land_altitude -= 5;
    sendAltitudeSpeedReferences(land_speed,land_altitude);
  }
	return false;
}

void BehaviorLandWithDF::sendAltitudeSpeedReferences(const double& dz_speed , const double& land_altitude){
  aerostack_msgs::TrajectoryWaypoints reference_waypoints;
  geometry_msgs::PoseStamped path_point;
  path_point.header.frame_id="odom";
  path_point.pose.position.x = (float)activationPosition.x;
  path_point.pose.position.y = (float)activationPosition.y;
  path_point.pose.position.z =(float)land_altitude;
  reference_waypoints.poses.emplace_back(path_point);
  reference_waypoints.header.frame_id="odom";
  reference_waypoints.header.stamp = ros::Time::now();
  reference_waypoints.yaw_mode = YAW_MODE;
  reference_waypoints.max_speed = dz_speed;
  waypoints_references_pub_.publish(reference_waypoints);
}

void BehaviorLandWithDF::poseCallback(const geometry_msgs::PoseStamped& _msg){
	position_=_msg.pose.position;
}

void BehaviorLandWithDF::statusCallBack(const aerostack_msgs::FlightState &msg){
  status_msg = msg;
}


void BehaviorLandWithDF::thrustCallBack(const mavros_msgs::Thrust& _msg){
  thrust_ = _msg.thrust;
}