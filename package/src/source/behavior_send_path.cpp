/*!********************************************************************************
 * \brief     Send path behavior implementation 
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

#include "../include/behavior_send_path.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorSendPath behavior;
  behavior.start();
  return 0;
}

BehaviorSendPath::BehaviorSendPath() : BehaviorExecutionManager(){ 
  setName("send_path");
  setExecutionGoal(ExecutionGoals::ACHIEVE_GOAL);
}

BehaviorSendPath::~BehaviorSendPath(){}

void BehaviorSendPath::onConfigure(){ 
  nh = getNodeHandle();
  nspace = getNamespace();
  ros_utils_lib::getPrivateParam<std::string>("~motion_reference_waypoints_path_topic"	  , motion_reference_waypoints_path_topic   ,"motion_reference/waypoints");
  ros_utils_lib::getPrivateParam<std::string>("~motion_reference_traj_topic"    , motion_reference_traj_topic_,     "motion_reference/trajectory");
  path_references_pub_ = nh.advertise<aerostack_msgs::TrajectoryWaypoints>("/" + nspace + "/" + motion_reference_waypoints_path_topic, 1, true);
  traj_sub_ = nh.subscribe("/" + nspace + "/" + motion_reference_traj_topic_, 1, &BehaviorSendPath::CallbackTrajectoryTopic,this);
}

void BehaviorSendPath::onActivate(){
  int yaw_mode = aerostack_msgs::TrajectoryWaypoints::PATH_FACING;
  double speed = 1;
  aerostack_msgs::TrajectoryWaypoints reference_waypoints;
  //Checks if path distance is not too long and argument is defined
  std::string arguments = getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  if(config_file["path"].IsDefined()){
    std::vector<std::vector<double>> points=config_file["path"].as<std::vector<std::vector<double>>>();
    for(int i=0;i<points.size();i++){
      geometry_msgs::PoseStamped path_point;
      path_point.header.frame_id="odom";
      path_point.pose.position.x = points[i][0];
      path_point.pose.position.y = points[i][1];
      path_point.pose.position.z = points[i][2];
      reference_waypoints.poses.emplace_back(path_point);
    }
  }else{
    setErrorMessage("Error: Path is not defined");
    std::cout<<"Error: Path is not defined"<<std::endl;
    return;
  }
  if(config_file["yaw_mode"].IsDefined()){
    yaw_mode = config_file["yaw_mode"].as<int>(); 
  }
  if(config_file["speed"].IsDefined()){
    speed = config_file["speed"].as<double>(); 
  }

  reference_waypoints.header.frame_id="odom";
  reference_waypoints.header.stamp = ros::Time::now();
  reference_waypoints.yaw_mode = yaw_mode;
  reference_waypoints.max_speed = speed;
  path_references_pub_.publish(reference_waypoints);
  moving = false;
}

void BehaviorSendPath::onDeactivate(){
  moving = false;
}

void BehaviorSendPath::onExecute(){
}

bool BehaviorSendPath::checkSituation(){
  return true;
}

void BehaviorSendPath::checkGoal(){
}

void BehaviorSendPath::checkProgress(){
}

void BehaviorSendPath::checkProcesses(){
}

void BehaviorSendPath::CallbackTrajectoryTopic(const trajectory_msgs::JointTrajectoryPoint& traj){
  if (moving && (traj.velocities[0]==0 && traj.velocities[1]==0 && traj.velocities[2]==0) && (traj.accelerations[0]==0 && traj.accelerations[1]==0 && traj.accelerations[2]==0)){
    BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
  }
  if (!moving && (traj.velocities[0]!=0 && traj.velocities[1]!=0 && traj.velocities[2]!=0) && (traj.accelerations[0]!=0 && traj.accelerations[1]!=0 && traj.accelerations[2]!=0)){
    moving = true;
  }
  
}