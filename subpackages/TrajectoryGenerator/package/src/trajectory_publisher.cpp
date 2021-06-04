#include "trajectory_publisher.hpp"


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

    
    static vector<double> pos(4);
    static vector<double> vel(4);
    static vector<double> acc(4);

    for(int i =0;i<pos.size();i++){
        pos[i] = refs[i][0];
        vel[i] = refs[i][1];
        acc[i] = refs[i][2];
    }

    traj_msgs.positions = pos;
    traj_msgs.velocities = vel;
    traj_msgs.accelerations = acc;    
    
    if (publish){
        traj_pub_.publish(traj_msgs);
    }
}


void TrajectoryPublisher::plotTrajectory(float period){
    
    std::array<std::array<float,3>,4> poses;
    std::vector<geometry_msgs::PoseStamped> pose_vec;
    nav_msgs::Path traj_path;
    
    auto current_time = ros::Time::now();
    float x,y,z;
    
    for (float t_ = 0; t_ < traj_gen_->getEndTime()+1;t_+=period ){

        traj_gen_->evaluateTrajectory(t_, poses);
        
        x = poses[0][0];
        y = poses[1][0];
        z = poses[2][0];
        
        geometry_msgs::PoseStamped traj_pose;

        traj_pose.header.frame_id = frame_id_;
        traj_pose.header.stamp = current_time;

        traj_pose.pose.position.x = x;
        traj_pose.pose.position.y = y;
        traj_pose.pose.position.z = z;

        traj_path.header.frame_id = "odom";
        traj_path.header.stamp = current_time;

        pose_vec.emplace_back(traj_pose);
    }
    
    traj_path.poses = pose_vec;
    path_pub_.publish(traj_path);

}


// CALLBACKS

void TrajectoryPublisher::CallbackWaypointsTopic(const std_msgs::Float32MultiArray& waypoints_msg){
    // clean waypoints

    std::vector<float> waypoints_x;
    std::vector<float> waypoints_y;
    std::vector<float> waypoints_z;
    std::vector<float> waypoints_yaw;

    auto n_waypoints = (int)waypoints_msg.data[0];
    float medium_speed = waypoints_msg.data[1];
     
    if (medium_speed <= 0.0)throw std::out_of_range("speed must be > 0.0 m/s");

    waypoints_x.reserve(n_waypoints+1);
    waypoints_y.reserve(n_waypoints+1);
    waypoints_z.reserve(n_waypoints+1);
    waypoints_yaw.reserve(n_waypoints+1);

    //add actual position to path
    // if (!(type_ == Trajectory_type::eth_spline_linear  || type_ == Trajectory_type::eth_spline_non_linear)){

    waypoints_x.emplace_back(actual_pose_[0]);
    waypoints_y.emplace_back(actual_pose_[1]);
    waypoints_z.emplace_back(actual_pose_[2]);
    waypoints_yaw.emplace_back(actual_pose_[3]);
    for(int i = 2; i < (n_waypoints*4 + 2);i = i+4){
        waypoints_x.emplace_back(waypoints_msg.data[i]);
        waypoints_y.emplace_back(waypoints_msg.data[i+1]);
        waypoints_z.emplace_back(waypoints_msg.data[i+2]);
        waypoints_yaw.emplace_back(waypoints_msg.data[i+3]);
    }
    n_waypoints+=1;
    
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
        is_trajectory_generated_ = traj_gen_->generateTrajectory(waypoints,medium_speed,actual_vel_acc_);
    }
    else{   
        is_trajectory_generated_ = traj_gen_->generateTrajectory(waypoints,medium_speed);
    }

    if (is_trajectory_generated_) {
        begin_time_ = ros::Time::now();
        plotTrajectory(0.2);
    }
}


void TrajectoryPublisher::CallbackPoseTopic(const geometry_msgs::PoseStamped &pose_msg){
    actual_pose_[0] = pose_msg.pose.position.x;
    actual_pose_[1] = pose_msg.pose.position.y;
    actual_pose_[2] = pose_msg.pose.position.z;
    // std::cout << "actual_pose" << actual_pose_[0] <<", " << actual_pose_[1] <<", " << actual_pose_[2] <<std::endl;
    
    auto quat = pose_msg.pose.orientation;
    double roll, pitch, yaw;
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 R(q);
    R.getRPY(roll, pitch, yaw);
    actual_pose_[3]=yaw;
}

void TrajectoryPublisher::CallbackSpeedTopic(const geometry_msgs::TwistStamped &twist_msg){
    auto dt = ros::Time::now() - last_time_;
    last_time_ = ros::Time::now();

    actual_vel_acc_[3] = (float) (twist_msg.twist.linear.x - actual_vel_acc_[0])/dt.toSec();
    actual_vel_acc_[4] = (float) (twist_msg.twist.linear.y - actual_vel_acc_[1])/dt.toSec();
    actual_vel_acc_[5] = (float) (twist_msg.twist.linear.z - actual_vel_acc_[2])/dt.toSec();
    actual_vel_acc_[0] = (float) twist_msg.twist.linear.x;
    actual_vel_acc_[1] = (float) twist_msg.twist.linear.y;
    actual_vel_acc_[2] = (float) twist_msg.twist.linear.z;
    
}