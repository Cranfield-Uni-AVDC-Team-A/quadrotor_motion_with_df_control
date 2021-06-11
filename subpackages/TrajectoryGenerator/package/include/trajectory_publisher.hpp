#ifndef __TRAJECTORY_PUBLISHER_H__
#define __TRAJECTORY_PUBLISHER_H__
#include "ros/ros.h" 
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include "aerostack_msgs/TrajectoryWaypoints.h"

#include <iostream>
#include <vector>
#include <math.h>
#include <string>

#include "trajectory_generator.hpp" 
#include "eth_traj_wrapper.hpp"
#include "ros_utils_lib/ros_utils.hpp"

#define LOG_(x) std::cout<< x << std::endl


enum Trajectory_type {spline_basic,circle,lemniscate,eth_spline_linear,eth_spline_non_linear};

class TrajectoryPublisher{
private:
    // Ros stuff
    ros::NodeHandle nh_;    
    std::string n_space_;
    std::string self_localization_pose_topic_;
    std::string self_localization_speed_topic_;
    std::string motion_reference_traj_topic_;
    std::string motion_reference_waypoints_path_topic_;
    std::string debug_traj_generated_topic_;

    ros::Subscriber waypoints_sub_;
    ros::Subscriber pose_sub_ ;
    ros::Subscriber speed_sub_;
        
    ros::Publisher traj_pub_;
    ros::Publisher path_pub_;


    TrajectoryGenerator *traj_gen_;

    bool is_trajectory_generated_ = false;
    ros::Time begin_time_;
    std::string frame_id_ = "odom"; 
    Trajectory_type type_;
    float actual_pose_[4] = {0.0f,0.0f,0.0f,0.0f};
    std::vector<float> actual_vel_acc_;
    ros::Time last_time_;

    int yaw_mode_=0;
    float begin_traj_yaw_ = 0.0f;

public:
    

    TrajectoryPublisher(Trajectory_type type); 
    ~TrajectoryPublisher();

    void setup();
    void run();

private:

    void plotTrajectory(float period);
    void publishTrajectory();
    void CallbackWaypointsTopic(const aerostack_msgs::TrajectoryWaypoints& );
    void CallbackPoseTopic(const geometry_msgs::PoseStamped &pose_msg);
    void CallbackSpeedTopic(const geometry_msgs::TwistStamped &twist_msg);

};

#endif