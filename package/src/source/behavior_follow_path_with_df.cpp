/*!********************************************************************************
 * \brief     follow_path implementation
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

#include "../include/behavior_follow_path_with_df.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorFollowPathWithDF behavior;
  behavior.start();
  return 0;
}

BehaviorFollowPathWithDF::BehaviorFollowPathWithDF() : BehaviorExecutionManager() { 
  setName("follow_path_with_df");
  setExecutionGoal(ExecutionGoals::KEEP_RUNNING);
}

BehaviorFollowPathWithDF::~BehaviorFollowPathWithDF() {}

void BehaviorFollowPathWithDF::onConfigure(){
  node_handle = getNodeHandle();
  nspace = getNamespace();

  ros_utils_lib::getPrivateParam<std::string>("~controllers_topic"	                      , command_high_level_str                  ,"actuator_command/flight_action");
  ros_utils_lib::getPrivateParam<std::string>("~status_topic"	                            , status_str                              ,"self_localization/flight_state");
  ros_utils_lib::getPrivateParam<std::string>("~motion_reference_traj_topic"              , motion_reference_traj_topic_            ,"motion_reference/trajectory");

  //Subscriber
  status_sub = node_handle.subscribe("/" + nspace + "/"+status_str, 1, &BehaviorFollowPathWithDF::statusCallBack, this);
}

bool BehaviorFollowPathWithDF::checkSituation(){
  //Quadrotor is FLYING
  if (status_msg.state == aerostack_msgs::FlightState::LANDED){
    setErrorMessage("Error: Drone is landed");
    return false;
  }
return true;
}

void BehaviorFollowPathWithDF::checkGoal(){
}

void BehaviorFollowPathWithDF::onExecute(){
  
}

void BehaviorFollowPathWithDF::checkProgress() {

}

void BehaviorFollowPathWithDF::onActivate(){
 //Subscribers
  //Publishers
  command_high_level_pub = node_handle.advertise<aerostack_msgs::FlightActionCommand>("/" + nspace + "/"+command_high_level_str, 1, true);

  //MOVE
  high_level_command.header.frame_id = "behavior_follow_path";
  high_level_command.action = aerostack_msgs::FlightActionCommand::MOVE;
  command_high_level_pub.publish(high_level_command);

  std::string arguments = getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  if(config_file["path"].IsDefined()){
    traj_sub_ = node_handle.subscribe("/" + nspace + "/" + motion_reference_traj_topic_, 1, &BehaviorFollowPathWithDF::CallbackTrajectoryTopic,this);
    setExecutionGoal(ExecutionGoals::ACHIEVE_GOAL);
  }
}

void BehaviorFollowPathWithDF::onDeactivate(){
  aerostack_msgs::FlightActionCommand msg;
  msg.header.frame_id = "behavior_follow_path";
  msg.action = aerostack_msgs::FlightActionCommand::HOVER;
  command_high_level_pub.publish(msg);
  command_high_level_pub.shutdown();
  path_blocked_sub.shutdown();
  traj_sub_.shutdown();
}

void BehaviorFollowPathWithDF::checkProcesses() { 
 
}

// Callbacks


void BehaviorFollowPathWithDF::statusCallBack(const aerostack_msgs::FlightState &msg){
  status_msg = msg;
}

void BehaviorFollowPathWithDF::CallbackTrajectoryTopic(const trajectory_msgs::JointTrajectoryPoint& traj){
  double value = 0.0f;
  static float previous_time = traj.time_from_start.toSec();
  if (traj.time_from_start.toSec() - previous_time < -0.001 ){
  }
  for (unsigned short int i =0; i<3; i++ ){
    value += fabs(traj.velocities[i]);
    value += fabs(traj.accelerations[i]);
  }
  if (value == 0.0f){
    BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
  }
  previous_time = traj.time_from_start.toSec();
    
}