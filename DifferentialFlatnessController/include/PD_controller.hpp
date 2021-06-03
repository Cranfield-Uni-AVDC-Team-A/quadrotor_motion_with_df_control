#ifndef __PD_CONTROLLER_H__
#define __PD_CONTROLLER_H__

//  ros
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "mav_msgs/RateThrust.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Range.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

#include "geometry_msgs/TwistStamped.h"

#include "mavros_msgs/Thrust.h"

// Eigen
#include <Eigen/Dense>

// Std libraries
#include <iostream>
#include <vector>
#include <math.h>
#include <array>

// dynamic reconfigure
#include <differential_flatness_controller/ControllerConfig.h>
#include <dynamic_reconfigure/server.h>
// own libraries 
#include "ros_utils_lib/ros_utils.hpp"
#include <robot_process.h>
// definitions 

#define DEBUG 0
#define LINEARIZATION 0
#define DYNAMIC_TUNING

using Vector3d = Eigen::Vector3d;


struct Control_flags{
    bool traj_generated;
    bool hover_position;
    bool state_received;
};

struct UAV_state{
    // State composed of s = [pose ,d_pose]'
    float pos[3];
    float rot[3];
    float vel[3];
    float omega[3];
};


class PD_controller : public RobotProcess{
private:

    std::string n_space_;
    std::string self_localization_pose_topic_;
    std::string self_localization_speed_topic_;
    std::string sensor_measurement_imu_topic_;
    std::string motion_reference_traj_topic_;
    
    std::string actuator_command_thrust_topic_;
    std::string actuator_command_speed_topic_;

    float mass = 1.0f;    

    
    ros::NodeHandle n;
    
    ros::Subscriber sub_traj; 
    ros::Subscriber sub_pose; 
    ros::Subscriber sub_speed; 
    ros::Subscriber sub_imu; 

    ros::Publisher command_pub;
    ros::Publisher thrust_pub;
    ros::Publisher speeds_pub;
    

    mav_msgs::RateThrust command_msg;
    
    UAV_state state_;
    Control_flags flags_;
    
    // controller stuff
    const float g = 9.81;
    const float angle_limit = 1.0; // pi/4 < value < pi/2 

    
    Eigen::Vector3d Kp_lin_;
    Eigen::Vector3d Kd_lin_;
    Eigen::Vector3d Kp_ang_;
    Eigen::Vector3d Ki_lin_;
    Eigen::Vector3d accum_error_;

    Eigen::Matrix3d Rot_matrix;
    
    float u1 = 0.0;
    float u2[3] = {0.0,0.0,0.0};

    std::array<std::array<float,3>,4> refs_;

public:
    PD_controller();
    ~PD_controller(){};

    void updateErrors();
    void computeActions();
    void publishActions();
    #ifdef DYNAMIC_TUNING
    void parametersCallback(pd_controller::ControllerConfig &config, uint32_t level);
    #endif
private: /*RobotProcess*/
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();
    

private:

    void followTrajectory();
    void hover();
    
    void CallbackTrajTopic(const trajectory_msgs::JointTrajectoryPoint& traj_msg);
    void CallbackOdomTopic(const nav_msgs::Odometry& odom_msg);

    void CallbackPoseTopic(const geometry_msgs::PoseStamped& pose_msg);
    void CallbackSpeedTopic(const geometry_msgs::TwistStamped& twist_msg);
    void CallbackImuTopic(const sensor_msgs::Imu& imu_msg);


};

#endif