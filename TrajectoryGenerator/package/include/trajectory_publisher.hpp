#ifndef __TRAJECTORY_PUBLISHER_H__
#define __TRAJECTORY_PUBLISHER_H__
#include "ros/ros.h" 
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <iostream>
#include <vector>
#include <math.h>
#include <string>

#include "trajectory_generator.hpp" 
#include "eth_traj_wrapper.hpp"

#define LOG_(x) std::cout<< x << std::endl


enum Trajectory_type {spline_basic,circle,lemniscate,eth_spline_linear,eth_spline_non_linear};

class TrajectoryPublisher{
private:
    // Ros stuff
    ros::NodeHandle nh_;    
    ros::Subscriber waypoints_sub_;
    ros::Subscriber sub_odom; 

    
    ros::Publisher traj_pub_;
    ros::Publisher path_pub_;
    


    TrajectoryGenerator *traj_gen_;

    bool is_trajectory_generated_ = false;

    ros::Time begin_time_;
    std::string frame_id_ = "world"; 
    Trajectory_type type_;
    float actual_pose_[4] = {0.0f,0.0f,0.0f,0.0f};
    std::vector<float> actual_vel_acc_;
    ros::Time last_time_;

public:
    

    TrajectoryPublisher(Trajectory_type type); 
    ~TrajectoryPublisher();

    void setup();
    void run();

private:

    void plotTrajectory(float period);
    void publishTrajectory();
    void CallbackWaypointsTopic(const std_msgs::Float32MultiArray& );
    void CallbackOdomTopic(const nav_msgs::Odometry& );



};

#endif