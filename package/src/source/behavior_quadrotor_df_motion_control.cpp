/*!********************************************************************************
 * \brief     quadrotor_df_motion_control implementation
 * \authors   Pablo Santamaria
<<<<<<< HEAD
 *            Miguel Fernandez Cortizas
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
=======
 * \copyright Copyright (c) 2021 Universidad Politecnica de Madrid
>>>>>>> 161ec29526e5c96169fd192582cb720e4cc854f4
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

#include "../include/behavior_quadrotor_df_motion_control.h"

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << "Node: " << ros::this_node::getName() << " started" << std::endl;
  BehaviorQuadrotorDFMotionControl behavior;
  behavior.start();
  return 0;
}

BehaviorQuadrotorDFMotionControl::BehaviorQuadrotorDFMotionControl() : BehaviorExecutionManager() { 
  setName("quadrotor_df_motion_control"); 
  setExecutionGoal(ExecutionGoals::KEEP_RUNNING); 
}

BehaviorQuadrotorDFMotionControl::~BehaviorQuadrotorDFMotionControl() {}

void BehaviorQuadrotorDFMotionControl::onConfigure()
{

  node_handle = getNodeHandle();
  nspace = getNamespace();
}

bool BehaviorQuadrotorDFMotionControl::checkSituation()
{
  return true;
}

void BehaviorQuadrotorDFMotionControl::onActivate()
{
  std::cout<<"ACTIVATED QUADROTOR DF CONTROLLER"<<std::endl;
  ros::ServiceClient start_controller=node_handle.serviceClient<std_srvs::Empty>("/"+nspace+"/differential_flatness_controller/start");
  std_srvs::Empty req;
  start_controller.call(req);
}

void BehaviorQuadrotorDFMotionControl::onDeactivate()
{
  std::cout<<"DEACTIVATED QUADROTOR DF CONTROLLER"<<std::endl;
  ros::ServiceClient stop_controller=node_handle.serviceClient<std_srvs::Empty>("/"+nspace+"/differential_flatness_controller/stop");
  std_srvs::Empty req;
  stop_controller.call(req);
}

void BehaviorQuadrotorDFMotionControl::onExecute()
{ 

}

void BehaviorQuadrotorDFMotionControl::checkGoal(){}


void BehaviorQuadrotorDFMotionControl::checkProgress() {

}


void BehaviorQuadrotorDFMotionControl::checkProcesses() 
{ 
 
}
