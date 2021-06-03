/*!********************************************************************************
 * \brief     Take off behavior implementation 
 * \authors   Pablo Santamaria
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
  motion_reference_path_pub = nh.advertise<nav_msgs::Path>("/" + nspace + "/motion_reference/path", 1, true);

}

void BehaviorSendPath::onActivate(){
  nav_msgs::Path reference_path;
  //Checks if path distance is not too long and argument is defined
  std::string arguments = getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  if(config_file["path"].IsDefined()){
    std::vector<std::vector<double>> points=config_file["path"].as<std::vector<std::vector<double>>>();
    geometry_msgs::PoseStamped path_point;
    float distance = 0;
    //First point
    path_point.pose.position.x = points[0][0];
    path_point.pose.position.y = points[0][1];
    path_point.pose.position.z = points[0][2];
    reference_path.poses.push_back(path_point);    
    for(int i=1;i<points.size();i++){
      path_point.pose.position.x = points[i][0];
      path_point.pose.position.y = points[i][1];
      path_point.pose.position.z = points[i][2];
      reference_path.poses.push_back(path_point);
      distance += abs(sqrt(pow(points[i-1][0]-points[i][0],2)+pow(points[i-1][1]-points[i][1],2)+pow(points[i-1][2]-points[i][2],2)));
    }
    if (distance > MAX_DISTANCE){
      setErrorMessage("Error: Path is too long");
      std::cout<<"Error: Path is too long"<<std::endl;
      return; 
    }
  }else{
    setErrorMessage("Error: Path is not defined");
    std::cout<<"Error: Path is not defined"<<std::endl;
    return;
  }
  motion_reference_path_pub.publish(reference_path);
  //BehaviorExecutionManager::setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
}

void BehaviorSendPath::onDeactivate(){
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