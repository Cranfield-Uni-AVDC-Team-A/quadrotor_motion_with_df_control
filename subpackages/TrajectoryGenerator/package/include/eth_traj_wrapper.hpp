#ifndef __ETH_TRAJ_WRAPPER_H__
#define __ETH_TRAJ_WRAPPER_H__

#include "ros/ros.h"

#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <cmath>

#include "trajectory_generator.hpp"
#include "mav_trajectory_generation/polynomial_optimization_linear.h"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include "mav_trajectory_generation/trajectory.h"
// Feasibility libraries
#include "mav_trajectory_generation_ros/feasibility_analytic.h"
#include "mav_trajectory_generation_ros/feasibility_base.h"
#include "mav_trajectory_generation_ros/feasibility_sampling.h"
#include "mav_trajectory_generation_ros/feasibility_recursive.h"
#include "mav_trajectory_generation_ros/input_constraints.h"


#include "nav_msgs/Path.h"


#include <mutex>
#include <thread>
#include <memory>

struct TrajConstraints{
    float g_acc_min,g_acc_max;
    float vel_max;
    float omega_pr_max,omega_yaw_max;
    float acc_yaw_max;
};
 
enum TrajGeneratorOptimizator {LINEAR, NONLINEAR};

class ETHSplineGenerator:public TrajectoryGenerator{
private :

    // const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::SNAP;
    // const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::JERK;
    const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::ACCELERATION;
    // const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::VELOCITY;

    const int dimension_ = 3;
    // const double a_max_ = 0.5*9.81f;
    const double a_max_ = 1*9.81f;

    TrajGeneratorOptimizator type_;
    TrajConstraints constraints_;


    // mav_trajectory_generation::Trajectory trajectory_;    

    std::unique_ptr<mav_trajectory_generation::Trajectory> traj_ptr_ = nullptr;

    std::mutex trajectory_mutex_;    
    std::mutex time_mutex_;
    std::thread gen_traj_thread_;
    std::thread plot_thread_;

    float delay_t_ = 0.0f;
    bool delay_t_assigned = false;

    ros::Time begin_time_;
    float yaw_measured_ = 0.0f;
    
    Eigen::Vector3d actual_pos_;

public:

    ETHSplineGenerator(TrajGeneratorOptimizator type);
    ~ETHSplineGenerator();
    void poseCallback(const geometry_msgs::PoseStamped& msg);

    bool generateTrajectory(const std::vector<std::vector<float>>& waypoints, float speed){return true;}
    bool generateTrajectory(const std::vector<std::vector<float>>& waypoints, float speed , const std::vector<float>& actual_speed_acc);
    bool evaluateTrajectory(float t , std::array<std::array<float,3>,4>& refs_);
    
    inline ros::Time getBeginTime(){
        time_mutex_.lock();
        auto out = begin_time_; 
        time_mutex_.unlock();
        return out;
    }
    bool getTrajectoryGenerated() {
        trajectory_mutex_.lock();
        bool out = trajectory_generated_;
        trajectory_mutex_.unlock();
        return out;
    }

private:
    
    void genTraj(const std::vector<std::vector<float>>& waypoints, float speed , const std::vector<float>& actual_speed_acc);
    bool checkTrajectoryFeasibility();

};


#endif