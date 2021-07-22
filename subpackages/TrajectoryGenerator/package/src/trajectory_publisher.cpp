#include "trajectory_publisher.hpp"


double extractYawFromQuat(const geometry_msgs::Quaternion& quat){
    double roll, pitch, yaw;
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 R(q);
    R.getRPY(roll, pitch, yaw);
    return yaw;
};

TrajectoryPublisher::TrajectoryPublisher(Trajectory_type type)
    :type_(type)
{
    switch(type){
        case Trajectory_type::circle: 
            traj_gen_= new CircleGenerator;
            break;
        case Trajectory_type::lemniscate:
            traj_gen_= new LemniscateGenerator;
            break;
        case Trajectory_type::eth_spline_linear:
            traj_gen_= new ETHSplineGenerator(TrajGeneratorOptimizator::LINEAR);  
            break;
        case Trajectory_type::eth_spline_non_linear:
            traj_gen_= new ETHSplineGenerator(TrajGeneratorOptimizator::NONLINEAR);
            break;
        default: throw std::invalid_argument("Trajectory type does not exist");  break;
    } 


    ros_utils_lib::getPrivateParam<std::string>("~namespace", n_space_, "drone1");
    ros_utils_lib::getPrivateParam<std::string>("~self_localization_pose_topic"   , self_localization_pose_topic_,    "self_localization/pose");
    ros_utils_lib::getPrivateParam<std::string>("~self_localization_speed_topic"  , self_localization_speed_topic_,   "self_localization/speed");
    ros_utils_lib::getPrivateParam<std::string>("~motion_reference_traj_topic"    , motion_reference_traj_topic_,     "motion_reference/trajectory");
    ros_utils_lib::getPrivateParam<std::string>("~motion_reference_waypoints_path_topic", motion_reference_waypoints_path_topic_, "motion_reference/waypoints");
    ros_utils_lib::getPrivateParam<std::string>("~debug_traj_generated_topic"           , debug_traj_generated_topic_, "debug/traj_generated");


    waypoints_sub_ = nh_.subscribe("/" + n_space_ + "/" + motion_reference_waypoints_path_topic_, 1, &TrajectoryPublisher::CallbackWaypointsTopic,this);
    pose_sub_      = nh_.subscribe("/" + n_space_ + "/" + self_localization_pose_topic_,1,&TrajectoryPublisher::CallbackPoseTopic,this);
    speed_sub_     = nh_.subscribe("/" + n_space_ + "/" + self_localization_speed_topic_,1,&TrajectoryPublisher::CallbackSpeedTopic,this);

    traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectoryPoint>("/" + n_space_ + "/" + motion_reference_traj_topic_, 1);
    path_pub_ = nh_.advertise<nav_msgs::Path >                      ("/" + n_space_ + "/" + debug_traj_generated_topic_, 1);

    actual_vel_acc_ = std::vector<float>(6);
    last_time_ = ros::Time::now();

}

TrajectoryPublisher::~TrajectoryPublisher(){
    delete traj_gen_;
}

void TrajectoryPublisher::setup(){

};

void TrajectoryPublisher::run(){
    if(is_trajectory_generated_){
        publishTrajectory();
    }

}

void TrajectoryPublisher::publishTrajectory(){

    static trajectory_msgs::JointTrajectoryPoint traj_msgs;
    static std::array<std::array<float,3>,4> refs;
    
    auto time = ros::Time().now() - traj_gen_->getBeginTime();
    bool publish  = traj_gen_->evaluateTrajectory(time.toSec(),refs);
    
    static std::array<std::array<float,3>,4> prev_refs = refs;

    switch (yaw_mode_){
        case aerostack_msgs::TrajectoryWaypoints::KEEP_YAW:{
            refs[3][0] = begin_traj_yaw_;
        }
            break;
        case aerostack_msgs::TrajectoryWaypoints::PATH_FACING :{
            static float prev_vx = refs[0][1];
            static float prev_vy = refs[1][1];
            if (fabs(refs[0][1]) > 0.01 || (refs[1][1])>0.01){
                refs[3][0] = -atan2f((double)refs[0][1],(double)refs[1][1])+M_PI/2.0f;
                prev_vx = refs[0][1];    
                prev_vy = refs[1][1];    
            }else{
                refs[3][0] = -atan2f((double)prev_vx,(double)prev_vy)+M_PI/2.0f;
            }
        }
            break;
        case aerostack_msgs::TrajectoryWaypoints::GENERATE_YAW_TRAJ :
            {refs[3][0] = 0.0f;
            std::cerr << "YAW MODE NOT IMPLEMENTED YET"<<std::endl;}
            break;
        
        default:{
                std::cerr << "YAW MODE NOT DEFINED"<<std::endl;
        }
            break;
    }
    
    static vector<double> pos(4);
    static vector<double> vel(4);
    static vector<double> acc(4);

    const float alpha = 0.2;
    for(int i =0;i<pos.size();i++){
        pos[i] = (1-alpha)* prev_refs[i][0] + alpha * refs[i][0];
        vel[i] = (1-alpha)* prev_refs[i][1] + alpha *refs[i][1];
        acc[i] = (1-alpha)* prev_refs[i][2] + alpha *refs[i][2];
    }
    prev_refs = refs;
    
    traj_msgs.positions = pos;
    traj_msgs.velocities = vel;
    traj_msgs.accelerations = acc;    
    traj_msgs.time_from_start = time;    
    
    if (publish){
        traj_pub_.publish(traj_msgs);
    }
}


void TrajectoryPublisher::plotTrajectory(float period){

    
    std::array<std::array<float,3>,4> poses;
    std::vector<geometry_msgs::PoseStamped> pose_vec;
    nav_msgs::Path traj_path;
    
    ros::Time current_time = ros::Time::now();
    float x,y,z;
    while (!traj_gen_->getTrajectoryGenerated()){
        ros::Duration(0.1).sleep();
    }
    
    for (float t_ = 0; t_ < traj_gen_->getEndTime()+1;t_+=period ){

        traj_gen_->evaluateTrajectory(t_, poses,true);
        
        x = poses[0][0];
        y = poses[1][0];
        z = poses[2][0];
        
        geometry_msgs::PoseStamped traj_pose;

        traj_pose.header.frame_id = frame_id_;
        traj_pose.header.stamp = current_time;

        traj_pose.pose.position.x = x;
        traj_pose.pose.position.y = y;
        traj_pose.pose.position.z = z;

        
        pose_vec.emplace_back(traj_pose);
    }

    traj_path.header.frame_id = frame_id_;
    traj_path.header.stamp = current_time;
    
    traj_path.poses = pose_vec;
    path_pub_.publish(traj_path);

}


// CALLBACKS

void TrajectoryPublisher::CallbackWaypointsTopic(const aerostack_msgs::TrajectoryWaypoints& waypoints_msg){
    
    // clean waypoints

    // aerostack_msgs::TrajectoryWaypoints msg;
    // msg.header.stamp = ros::Time::now();
    // msg.header.frame_id = "odom";
    // msg.yaw_mode = waypoints_msg.yaw_mode;
    // msg.max_speed = waypoints_msg.max_speed;

    // msg.poses.clear();
    // msg.poses.reserve(waypoints_msg.poses.size()+1);
    // for (auto& elem:waypoints_msg.poses){
    //     msg.poses.emplace_back(elem);
    // }

    std::vector<float> waypoints_x;
    std::vector<float> waypoints_y;
    std::vector<float> waypoints_z;
    std::vector<float> waypoints_yaw;

    unsigned int n_waypoints = waypoints_msg.poses.size()+1;
    float max_speed = waypoints_msg.max_speed;
    yaw_mode_ = waypoints_msg.yaw_mode;
    frame_id_ = waypoints_msg.header.frame_id;
     
    if (max_speed <= 0.0)throw std::out_of_range("speed must be > 0.0 m/s");

    waypoints_x.reserve(n_waypoints);
    waypoints_y.reserve(n_waypoints);
    waypoints_z.reserve(n_waypoints);
    waypoints_yaw.reserve(n_waypoints);

    //add actual position to path
    
    waypoints_x.emplace_back(actual_pose_[0]);
    waypoints_y.emplace_back(actual_pose_[1]);
    waypoints_z.emplace_back(actual_pose_[2]);
    waypoints_yaw.emplace_back(actual_pose_[3]);

    

    for(auto &elem:waypoints_msg.poses){
        waypoints_x.emplace_back(elem.pose.position.x);
        waypoints_y.emplace_back(elem.pose.position.y);
        waypoints_z.emplace_back(elem.pose.position.z);
        waypoints_yaw.emplace_back(extractYawFromQuat(elem.pose.orientation));
    }
    
    #if DEBUG_TRAJ == 2
        std::cout << "New checkPoints adquired: \n";
        std::cout << "x:{";
        for (auto elem: waypoints_x) std::cout <<elem<<",";
        std::cout << "} \n";

        std::cout << "y:{";
        for (auto elem: waypoints_y) std::cout <<elem<<",";
        std::cout << "} \n";

        std::cout << "z:{";
        for (auto elem: waypoints_z) std::cout <<elem<<",";
        std::cout << "} \n";

        std::cout << "yaw:{";
        for (auto elem: waypoints_yaw) std::cout <<elem<<",";
        std::cout << "} \n";
    #endif

    std::vector<std::vector<float>> waypoints = {waypoints_x,waypoints_y,waypoints_z,waypoints_yaw};
    if (type_ == Trajectory_type::eth_spline_linear  || type_ == Trajectory_type::eth_spline_non_linear){
        is_trajectory_generated_ = traj_gen_->generateTrajectory(waypoints,max_speed,actual_vel_acc_);
    }
    else{   
        is_trajectory_generated_ = traj_gen_->generateTrajectory(waypoints,max_speed);
        
    }

    if (is_trajectory_generated_) {
        begin_traj_yaw_ = actual_pose_[3];
        begin_time_ = ros::Time::now();
        plotTrajectory(0.4);
    }
}


void TrajectoryPublisher::CallbackPoseTopic(const geometry_msgs::PoseStamped &pose_msg){
    actual_pose_[0] = pose_msg.pose.position.x;
    actual_pose_[1] = pose_msg.pose.position.y;
    actual_pose_[2] = pose_msg.pose.position.z;
    // std::cout << "actual_pose" << actual_pose_[0] <<", " << actual_pose_[1] <<", " << actual_pose_[2] <<std::endl;
    
    actual_pose_[3]=extractYawFromQuat(pose_msg.pose.orientation);
}

void TrajectoryPublisher::CallbackSpeedTopic(const geometry_msgs::TwistStamped &twist_msg){
    auto dt = ros::Time::now() - last_time_;
    last_time_ = ros::Time::now();
    static std::vector<float> last_refs(6,0.0f);
    const float alpha = 0.2;
    actual_vel_acc_[3] = (float) last_refs[3]*(1-alpha) + alpha * (twist_msg.twist.linear.x - actual_vel_acc_[0])/dt.toSec();
    actual_vel_acc_[4] = (float) last_refs[4]*(1-alpha) + alpha * (twist_msg.twist.linear.y - actual_vel_acc_[1])/dt.toSec();
    actual_vel_acc_[5] = (float) last_refs[5]*(1-alpha) + alpha * (twist_msg.twist.linear.z - actual_vel_acc_[2])/dt.toSec();
    actual_vel_acc_[0] = (float) last_refs[0]*(1-alpha) + alpha * twist_msg.twist.linear.x;
    actual_vel_acc_[1] = (float) last_refs[1]*(1-alpha) + alpha * twist_msg.twist.linear.y;
    actual_vel_acc_[2] = (float) last_refs[2]*(1-alpha) + alpha * twist_msg.twist.linear.z; 
 
    for (short int i= 0; i<actual_vel_acc_.size();i++){
        last_refs[i]=actual_vel_acc_[i];
    }
    
}