/*!********************************************************************************
 * \brief     land behavior implementation 
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

  pose_sub_ = nh.subscribe("/" + nspace + "/" + estimated_pose_topic ,1,&BehaviorLandWithDF::poseCallback,this);
  status_sub = nh.subscribe("/" + nspace + "/"+status_topic, 1, &BehaviorLandWithDF::statusCallBack, this);

  path_references_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/" + nspace + "/" + motion_reference_waypoints_path_topic, 1);
  flight_state_pub = nh.advertise<aerostack_msgs::FlightState>("/" + nspace + "/" + status_topic, 1);
  flight_action_pub = nh.advertise<aerostack_msgs::FlightActionCommand>("/" + nspace + "/" + flight_action_topic, 1);
}

void BehaviorLandWithDF::onActivate(){
  lastAltitude = ros::Time::now();
  aerostack_msgs::FlightActionCommand msg;
  msg.header.stamp = ros::Time::now();
  msg.action = aerostack_msgs::FlightActionCommand::LAND;
  flight_action_pub.publish(msg);

  activationPosition = position_;
  t_activacion_ = ros::Time::now();
  
}

void BehaviorLandWithDF::onDeactivate(){
  aerostack_msgs::FlightActionCommand msg;
  msg.header.stamp = ros::Time::now();
  msg.action = aerostack_msgs::FlightActionCommand::HOVER;
  flight_action_pub.publish(msg);
}

void BehaviorLandWithDF::onExecute(){
  //std::cout<<"1"<<std::endl;
  static bool doOnce = true;
  if(doOnce && (ros::Time::now()-t_activacion_).toSec()>1){
    sendAltitudeSpeedReferences(LAND_SPEED);
    doOnce = false;
  }
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
    msg.state = aerostack_msgs::FlightState::FLYING;
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
    std::list<float>::iterator it;
    float max = *altitudes_list.begin();
    float min = *altitudes_list.begin();
    for(it = altitudes_list.begin(); it != altitudes_list.end(); it++){
      avg += *it;
      if(*it < min){
        min = *it;
      }
      if(*it > max){
        max = *it;
      }
    }
    avg /= altitudes_list.size();
    if((max-min)<LAND_SPEED){
      BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
	    return true;
    }
  }
	return false;
}

void BehaviorLandWithDF::sendAltitudeSpeedReferences(const double& dz_speed , const double land_altitude){
  std_msgs::Float32MultiArray path;
  path.data = {1.0 , LAND_SPEED, (float)activationPosition.x , (float)activationPosition.y , (float)land_altitude, 0.0};
  path_references_pub_.publish(path);
}

void BehaviorLandWithDF::poseCallback(const geometry_msgs::PoseStamped& _msg){
	position_=_msg.pose.position;
}

void BehaviorLandWithDF::statusCallBack(const aerostack_msgs::FlightState &msg){
  status_msg = msg;
}