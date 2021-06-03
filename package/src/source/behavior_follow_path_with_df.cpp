/*!********************************************************************************
 * \brief     follow_path implementation
 * \authors   Alberto Rodelgo
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
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

void BehaviorFollowPathWithDF::onConfigure()
{
  node_handle = getNodeHandle();
  nspace = getNamespace();

  ros::param::get("~estimated_speed_topic", self_localization_speed_str);
  ros::param::get("~estimated_pose_topic", self_localization_pose_str);
  ros::param::get("~controllers_topic", command_high_level_str);
  ros::param::get("~status_topic", status_str);
  ros::param::get("~path_blocked_topic", path_blocked_topic_str);

  //Subscriber
  status_sub = node_handle.subscribe("/" + nspace + "/"+status_str, 1, &BehaviorFollowPathWithDF::statusCallBack, this);
}

bool BehaviorFollowPathWithDF::checkSituation()
{
  //Quadrotor is FLYING
  if (status_msg.state == aerostack_msgs::FlightState::LANDED){
    setErrorMessage("Error: Drone is landed");
    return false;
  }
return true;
}

void BehaviorFollowPathWithDF::checkGoal(){ 
  if(initiated && remaining_points == 0){
    //ros::Duration(5).sleep();
    initiated = false;
    BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
  } 
}

void BehaviorFollowPathWithDF::onExecute()
{
  
}

void BehaviorFollowPathWithDF::checkProgress() {
  if(path_blocked){
    path_blocked=false;
    path_blocked_sub.shutdown();
    BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
  }
  if (!execute) BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);

  /*//Quadrotor is too far from the target and it is not moving
  last_target_pose = current_target_pose;
  float targets_distance = abs(sqrt(pow(last_target_pose.pose.position.x-current_target_pose.pose.position.x,2)+pow(last_target_pose.pose.position.y-current_target_pose.pose.position.y,2)+pow(last_target_pose.pose.position.z-current_target_pose.pose.position.z,2)));
  float quadrotor_distance = abs(sqrt(pow(current_target_pose.pose.position.x-estimated_pose_msg.pose.position.x,2)+pow(current_target_pose.pose.position.y-estimated_pose_msg.pose.position.y,2)+pow(current_target_pose.pose.position.z-estimated_pose_msg.pose.position.z,2)));
  if(remaining_points > 0 && targets_distance > quadrotor_distance * 2 && checkQuadrotorStopped()) BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
  */
}

void BehaviorFollowPathWithDF::onActivate()
{
 //Subscribers
  self_localization_speed_sub = node_handle.subscribe("/" + nspace + "/"+self_localization_speed_str, 1, &BehaviorFollowPathWithDF::selfLocalizationSpeedCallBack, this);
  self_localization_pose_sub = node_handle.subscribe("/" + nspace + "/"+self_localization_pose_str, 1, &BehaviorFollowPathWithDF::selfLocalizationPoseCallBack, this);
  
  path_blocked_sub = node_handle.subscribe("/" + nspace + "/"+path_blocked_topic_str, 1, &BehaviorFollowPathWithDF::pathBlockedCallBack, this);
  //Publishers
  command_high_level_pub = node_handle.advertise<aerostack_msgs::FlightActionCommand>("/" + nspace + "/"+command_high_level_str, 1, true);

  remaining_points = 0;
  initiated = false;
  execute = true;
  path_blocked=false;

  //MOVE
  high_level_command.header.frame_id = "behavior_follow_path";
  high_level_command.action = aerostack_msgs::FlightActionCommand::MOVE;
  command_high_level_pub.publish(high_level_command);

  //Publisher
  path_references_pub_ = node_handle.advertise<std_msgs::Float32MultiArray>("/drone/waypoints", 1);
  motion_reference_path_sub = node_handle.subscribe("/" + nspace + "/motion_reference/path", 1, &BehaviorFollowPathWithDF::pathCallBack, this);
}

void BehaviorFollowPathWithDF::onDeactivate()
{
  aerostack_msgs::FlightActionCommand msg;
  msg.header.frame_id = "behavior_follow_path";
  msg.action = aerostack_msgs::FlightActionCommand::HOVER;
  command_high_level_pub.publish(msg);

  self_localization_speed_sub.shutdown();
  command_high_level_pub.shutdown();
  path_blocked_sub.shutdown();
  path_references_pub_.shutdown();
}

bool BehaviorFollowPathWithDF::checkQuadrotorStopped()
{
  if (received_speed){
    if (abs(estimated_speed_msg.twist.linear.x) <= 0.30 && abs(estimated_speed_msg.twist.linear.y) <= 0.30 && abs(estimated_speed_msg.twist.linear.z) <= 0.15){
        return true;
    }else{
      return false;
    }    
  }else{
    return false;
  }
}

void BehaviorFollowPathWithDF::checkProcesses() 
{ 
 
}


// Callbacks
void BehaviorFollowPathWithDF::pathBlockedCallBack(const std_msgs::Bool &msg){
  path_blocked = msg.data;
}

void BehaviorFollowPathWithDF::selfLocalizationSpeedCallBack(const geometry_msgs::TwistStamped &msg){ 
  estimated_speed_msg = msg; 
  received_speed = true;
}
void BehaviorFollowPathWithDF::selfLocalizationPoseCallBack(const geometry_msgs::PoseStamped &msg){
  estimated_pose_msg = msg;
}

void BehaviorFollowPathWithDF::statusCallBack(const aerostack_msgs::FlightState &msg){
  status_msg = msg;
}

void BehaviorFollowPathWithDF::pathCallBack(const nav_msgs::Path &msgPath){
  std_msgs::Float32MultiArray path;
  path.data.push_back(msgPath.poses.size());
  path.data.push_back(0.3);
  for(int i = 0; i<msgPath.poses.size(); i++){
    path.data.push_back(msgPath.poses[i].pose.position.x);
    path.data.push_back(msgPath.poses[i].pose.position.y);
    path.data.push_back(msgPath.poses[i].pose.position.z);
    path.data.push_back(0.0);
  }
  path_references_pub_.publish(path);
}
