#include "PD_controller.hpp"

//auxiliary functions
void printrefs(const std::array<std::array<float,3>,4>& refs_);
template <typename T> void SetZeros(Eigen::DenseBase<T>& m);


PD_controller::PD_controller(/* args */)
{
    
    ros_utils_lib::getPrivateParam<std::string>("~namespace", n_space_, "drone1");
    ros_utils_lib::getPrivateParam<float>("~uav_mass", mass, 1.0f);
    
    ros_utils_lib::getPrivateParam<std::string>("~self_localization_pose_topic" ,  self_localization_pose_topic_,    "self_localization/pose");
    ros_utils_lib::getPrivateParam<std::string>("~self_localization_speed_topic" , self_localization_speed_topic_,   "self_localization/speed");
    ros_utils_lib::getPrivateParam<std::string>("~sensor_measurement_imu_topic" ,  sensor_measurement_imu_topic_,    "sensor_measurement/imu");
    
    ros_utils_lib::getPrivateParam<std::string>("~motion_reference_traj_topic"  ,  motion_reference_traj_topic_,     "motion_reference/trajectory");
    ros_utils_lib::getPrivateParam<std::string>("~actuator_command_thrust_topic",  actuator_command_thrust_topic_,   "actuator_command/thrust");
    ros_utils_lib::getPrivateParam<std::string>("~actuator_command_speed_topic" ,  actuator_command_speed_topic_,    "actuator_command/speed");
    

    sub_pose  = n.subscribe("/" + n_space_ + "/" + self_localization_pose_topic_  ,1,&PD_controller::CallbackPoseTopic,this);
    sub_speed = n.subscribe("/" + n_space_ + "/" + self_localization_speed_topic_ ,1,&PD_controller::CallbackSpeedTopic,this);
    sub_imu   = n.subscribe("/" + n_space_ + "/" + sensor_measurement_imu_topic_  ,1,&PD_controller::CallbackImuTopic,this);

    sub_traj =n.subscribe("/" + n_space_ + "/" + motion_reference_traj_topic_ ,1,&PD_controller::CallbackTrajTopic,this);

    thrust_pub = n.advertise<mavros_msgs::Thrust>         ("/" + n_space_ + "/" + actuator_command_thrust_topic_, 1);
    speeds_pub = n.advertise<geometry_msgs::TwistStamped> ("/" + n_space_ + "/" + actuator_command_speed_topic_ , 1);

}


void PD_controller::ownSetUp(){

    #if LINEARIZATION == 1
        std::cout << "PD controller linearized" << std::endl;
    #elif LINEARIZATION == 0
        std::cout << "PD controller non-linearized" << std::endl;    
        std::cout << "uav_mass = " << mass << std::endl;    
    #endif

    Kp_lin_ << 4.5,4.5,5.0;
    Kd_lin_ << 1.0,1.0,2.0;
    Kp_ang_ << 5.0,5.0,5.0;    
    
    // Ki_lin_ << 0.0001,0.0001,0.00001;
    Ki_lin_ << 0.0,0.0,0.001;
    accum_error_ << 0,0,0;
    
    flags_.traj_generated = false;
    flags_.hover_position = false;
    flags_.state_received = false;

    //set all refs to zefs
    for (auto dof:refs_){
        for (auto elem:dof) elem = 0.0f;
    }
    //this->start();
}
  
void PD_controller::ownStart(){
}

void PD_controller::ownStop(){
}

void PD_controller::computeActions(){

#if LINEARIZATION == 0
    
    Vector3d r(state_.pos[0],state_.pos[1],state_.pos[2]);
    Vector3d rdot(state_.vel[0],state_.vel[1],state_.vel[2]);
    Vector3d r_t(refs_[0][0],refs_[1][0],refs_[2][0]);
    Vector3d rdot_t(refs_[0][1],refs_[1][1],refs_[2][1]);
    Vector3d rddot_t(refs_[0][2],refs_[1][2],refs_[2][2]);

    Vector3d F_des;
    Vector3d e_p = r - r_t;

    // std::cout <<"Position error: \n" <<e_p << std::endl;

    Vector3d e_v = rdot - rdot_t;
    
    Eigen::Matrix3d Kp_lin_mat = Kp_lin_.asDiagonal(); 
    Eigen::Matrix3d Kd_lin_mat = Kd_lin_.asDiagonal(); 
    Eigen::Matrix3d Kp_ang_mat = Kp_ang_.asDiagonal(); 
    Eigen::Matrix3d Ki_lin_mat = Ki_lin_.asDiagonal(); 

    // antiwindup

    float antiwindup_cte = 5.0f;
    for (short j = 0; j<3;j++){
        float antiwindup_value = antiwindup_cte/Ki_lin_[j];
        accum_error_[j] = (accum_error_[j] > antiwindup_value)? antiwindup_value: accum_error_[j];
        accum_error_[j] = (accum_error_[j] < -antiwindup_value)? -antiwindup_value: accum_error_[j];
    }

    F_des  = -Kp_lin_mat * e_p - Ki_lin_mat * accum_error_ -Kd_lin_mat * e_v + (mass*g) * Eigen::Vector3d(0,0,1) + mass * rddot_t;
    // F_des  = -Kp_lin_mat * e_p - Kd_lin_mat * e_v + (mass*g) * Eigen::Vector3d(0,0,1) + mass * rddot_t;

    Vector3d zb_des = F_des.normalized();
    Vector3d xc_des(cos(refs_[3][0]),sin(refs_[3][0]),0);
    Vector3d yb_des = zb_des.cross(xc_des).normalized();
    Vector3d xb_des = yb_des.cross(zb_des).normalized();

    Eigen::Matrix3d R_des;
    R_des.col(0)= xb_des;
    R_des.col(1)= yb_des;
    R_des.col(2)= zb_des;

    Eigen::Matrix3d Mat_e_rot = (R_des.transpose()*Rot_matrix -Rot_matrix.transpose()*R_des) ;
    
    Vector3d V_e_rot(Mat_e_rot(2,1),Mat_e_rot(0,2),Mat_e_rot(1,0));
    Vector3d E_rot = (1.0f/2.0f) * V_e_rot;
    

    // std::cout << std::endl<<"rot_matrix" <<Rot_matrix.col(2) << std::endl;
    
    u1 = (float) F_des.dot(Rot_matrix.col(2).normalized());
    
    // std::cout <<"u1 (N) : " << u1 << std::endl;

    Vector3d outputs = -Kp_ang_mat * E_rot; 

    u2[0] = outputs(0);
    u2[1] = outputs(1);
    u2[2] = outputs(2);
   
    
#elif LINEARIZATION == 1

    static float pos_error[3] = {0.0f,0.0f,0.0f};     
    static float sum_error[3] = {0.0f,0.0f,0.0f};     
    static float Ki_lin_[3] = {0.0f,0.0f,0.0f};

    pos_error[0] = refs_[0][0] - state_.pos[0];
    pos_error[1] = refs_[1][0] - state_.pos[1];
    pos_error[2] = refs_[2][0] - state_.pos[2];
    
    sum_error[0] += pos_error[0];
    sum_error[1] += pos_error[1]; 
    sum_error[2] += pos_error[2];
    
            
    float acc_x_des = refs_[0][2] + Kd_lin_[0] * (refs_[0][1] - state_.vel[0]) + Kp_lin_[0]*(pos_error[0]) + Ki_lin_[0] * sum_error[0];
    float acc_y_des = refs_[1][2] + Kd_lin_[1] * (refs_[1][1] - state_.vel[1]) + Kp_lin_[1]*(pos_error[1]) + Ki_lin_[1] * sum_error[1];
    float acc_z_des = refs_[2][2] + Kd_lin_[2] * (refs_[2][1] - state_.vel[2]) + Kp_lin_[2]*(pos_error[2]) + Ki_lin_[2] * sum_error[2];
    

    float psi_des = refs_[3][0]; 
    
    float phi_des = (1.0/(float)g) * (acc_x_des * sin(psi_des) -  acc_y_des * cos(psi_des));
    float theta_des = (1.0/(float)g) * (acc_x_des * cos(psi_des) +  acc_y_des * sin(psi_des));
    
    // check max angles
    if (phi_des > angle_limit) phi_des = angle_limit;
    else if (phi_des < -angle_limit) phi_des = -angle_limit;
    
    if (theta_des > angle_limit) theta_des = angle_limit;
    else if (theta_des < -angle_limit) theta_des = -angle_limit;
    std::array<std::array<float,3>,4> refs_;
    
    #if DEBUG == 4
        std::cout <<"acc_x_des: " << acc_x_des << std::endl;
        std::cout <<"acc_y_des: " << acc_y_des << std::endl;
        std::cout <<"acc_z_des: " << acc_z_des << std::endl;
        std::cout <<"phi_des: " << phi_des << std::endl;
        std::cout <<"theta_des: " << theta_des << std::endl;
        std::cout <<"psi_des: " << psi_des <<std::endl;
    #endif
    
    const float p_des = 0;
    const float q_des = 0; 
    const float r_des = 0;

    float d_phi_c =  p_des + Kp_ang_[0]*(phi_des - state_.rot[0]); 
    float d_theta_c = q_des + Kp_ang_[1]*(theta_des - state_.rot[1]);
    float d_psi_c = r_des + Kp_ang_[2]*(psi_des - state_.rot[2]);

    u1 = mass * (g + acc_z_des);

    u2[0] = d_phi_c;
    u2[1] = d_theta_c;
    u2[2] = d_psi_c;

#endif

    #if DEBUG == 1
    // std::cout << "u1  " << u1 << std::endl;
    // std::cout << "u2_1  " << u2[0] << std::endl;
    // std::cout << "u2_2  " << u2[0] << std::endl;
    // std::cout << "u2_3  " << u2[0] << std::endl;
    #endif

};

void PD_controller::publishActions(){
    command_msg.thrust.z = u1;
    command_msg.angular_rates.x = u2[0];
    command_msg.angular_rates.y = u2[1];
    command_msg.angular_rates.z = u2[2];

    static mavros_msgs::Thrust thrust_msg;
    static geometry_msgs::TwistStamped speeds_msg;
    
    thrust_msg.thrust= u1;
    speeds_msg.twist.angular.x = -u2[1];
    speeds_msg.twist.angular.y = u2[0];
    speeds_msg.twist.angular.z = u2[2];
    
    thrust_pub.publish(thrust_msg);
    speeds_pub.publish(speeds_msg);
    
};


void PD_controller::hover(){

    if (!flags_.hover_position){
        if (flags_.state_received){
            
            refs_[0][0] = state_.pos[0];
            refs_[1][0] = state_.pos[1];
            refs_[2][0] = state_.pos[2];
            refs_[3][0] = state_.rot[2];
            
            for (int j =0 ; j<4;j++){
                for (int i =1;i<3;i++) refs_[j][i]= 0; // reset only vels and accs
            }
                
            flags_.hover_position = true;
            std::cout << "hover State at: "<<"  x:"<< refs_[0][0]<<"  y:"<< refs_[1][0]<<"  z:"<< refs_[2][0]<<": True"<<std::endl;
        }
    }else{

        this->computeActions();
        this->publishActions();   
    }
}


void PD_controller::followTrajectory(){

        this->computeActions();
        this->publishActions();

}

void PD_controller::ownRun(){
    if (flags_.traj_generated){
        followTrajectory();
    }
    else{
        hover();
    }
        
};

/* --------------------------- CALLBACKS ---------------------------*/

#ifdef DYNAMIC_TUNING
    void PD_controller::parametersCallback(pd_controller::ControllerConfig &config, uint32_t level)
    {
        Kp_lin_ << config.Kp_x,config.Kp_y,config.Kp_z;
        Kd_lin_ << config.Kd_x,config.Kd_y,config.Kd_z;
        Kp_ang_ << config.Kp_roll,config.Kp_pitch,config.Kp_yaw;    
    }
#endif

void PD_controller::CallbackTrajTopic(const trajectory_msgs::JointTrajectoryPoint& traj_msg){
    flags_.traj_generated = true;
    for (int i=0;i<4;i++){
          refs_[i][0] = traj_msg.positions[i];
          refs_[i][1] = traj_msg.velocities[i];
          refs_[i][2] = traj_msg.accelerations[i];      
    }
  
}


void PD_controller::CallbackPoseTopic(const geometry_msgs::PoseStamped& pose_msg){

    state_.pos[0] = pose_msg.pose.position.x;
    state_.pos[1] = pose_msg.pose.position.y;
    state_.pos[2] = pose_msg.pose.position.z;
    auto quat = pose_msg.pose.orientation;
    double roll, pitch, yaw;
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 R(q);

    #if LINEARIZATION == 0
        for (int i = 0; i<3;i++)
            for (int j = 0; j<3;j++)
                Rot_matrix(i,j) = R[i][j];
    #endif

    R.getRPY(roll, pitch, yaw);
    state_.rot[0]=roll;
    state_.rot[1]=pitch;
    state_.rot[2]=yaw;

    flags_.state_received = true;

    #if DEBUG == 1    
        std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
    #endif
}



void PD_controller::CallbackSpeedTopic(const geometry_msgs::TwistStamped& twist_msg){

    state_.vel[0]=twist_msg.twist.linear.x;
    state_.vel[1]=twist_msg.twist.linear.y;
    state_.vel[2]=twist_msg.twist.linear.z;


}

void PD_controller::CallbackImuTopic(const sensor_msgs::Imu& imu_msg){

    state_.omega[0]=imu_msg.angular_velocity.x;
    state_.omega[1]=imu_msg.angular_velocity.y;
    state_.omega[2]=imu_msg.angular_velocity.z;

    #if DEBUG == 1    
        std::cout << "d_Roll: " << state_.omega[0] << ", d_Pitch: " << state_.omega[1] << ", d_Yaw: " << state_.omega[2] << std::endl;
    #endif


}


// Auxiliary functions

void printrefs(const std::array<std::array<float,3>,4>& refs_){
        int i  = 0;
        for (auto dof:refs_){
            std::cout << "refs_["<< i++<<"]:[";
            for (auto elem:dof){
                std::cout << elem <<" , ";
            }
            std::cout << "]\n";
        }
}

template <typename T>
void SetZeros(Eigen::DenseBase<T>& m){
    for (int i = 0 ;i < m.rows();i++)
        for (int j = 0 ;j < m.cols();j++)
            m(i,j) = 0.0f;

}
