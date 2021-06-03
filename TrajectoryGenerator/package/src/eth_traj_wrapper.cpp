#include "eth_traj_wrapper.hpp"


ETHSplineGenerator::ETHSplineGenerator(TrajGeneratorOptimizator type) :type_(type){
        
    /* * Constraints:
    *  Acc       = [g_acc_min*9.8, g_acc_max*9.8] (m/s/s)
    *  Vel       = [0 ,  Vel_max] (m/s)
    *  Omega_pr  = [0 , Omega_pr_max] (rad/s)
    *  Omega_yaw = [0 , Omega_yaw_max] (rad/s)
    *  acc_yaw   = [0 , acc_yaw_max] (rad/s/s)
    * */

    constraints_.g_acc_min     = 0.25f;
    constraints_.g_acc_max     = 4.0f;
    // constraints_.g_acc_max     = 2.0f;
    // constraints_.vel_max       = 5.0f;
    constraints_.vel_max       = 10.0f; 
    constraints_.omega_pr_max  = M_PI / 2.0f;
    constraints_.omega_yaw_max = M_PI / 2.0f;
    constraints_.acc_yaw_max   = M_PI;
    begin_time_=ros::Time::now();
};


ETHSplineGenerator::~ETHSplineGenerator(){
    gen_traj_thread_.join();
};


void ETHSplineGenerator::genTraj(const std::vector<std::vector<float>>& waypoints, float speed , const std::vector<float>& actual_speed_acc)
{

    ros::Time t_i  = ros::Time::now(); 

 // Waypoints[i][j] =>> i: (x,y,z,yaw) j:(waypoint_j)

    int n_points = waypoints[0].size();
    
    std::vector<mav_trajectory_generation::Vertex> vertices(n_points,dimension_);
    vertices[0].addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(actual_speed_acc[0],actual_speed_acc[1],actual_speed_acc[2]));
    vertices[0].addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(actual_speed_acc[3],actual_speed_acc[4],actual_speed_acc[5]));

    
    for (int i=0;i < n_points;i++){
        vertices[i].addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(waypoints[0][i],waypoints[1][i],waypoints[2][i]));
    }
    vertices[n_points-1].addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0,0,0));
    vertices[n_points-1].addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(0,0,0));

    float v_x = (waypoints[0][n_points-2]-waypoints[0][n_points-1]);
    float v_y = (waypoints[1][n_points-2]-waypoints[1][n_points-1]);
    float v_z = (waypoints[2][n_points-2]-waypoints[2][n_points-1]);
    //Calculate time slots

    #ifdef FAST
    double v_max = speed * 10;
    auto segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices,v_max,a_max_*6);
    #else
    
    double v_max = speed * 2;
    auto segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices,v_max,a_max_);
    
    #endif

    const int N = 10;
    mav_trajectory_generation::Segment::Vector segments;

    std::unique_ptr<mav_trajectory_generation::Trajectory> trajectory (new mav_trajectory_generation::Trajectory ());

 
    //Optimizer
    if (type_ == TrajGeneratorOptimizator::LINEAR){
        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension_);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
        opt.solveLinear();
        opt.getSegments(&segments);
        opt.getTrajectory((mav_trajectory_generation::Trajectory *)trajectory.get());
    }

    else if(type_ == TrajGeneratorOptimizator::NONLINEAR){
        mav_trajectory_generation::NonlinearOptimizationParameters parameters;
        parameters.max_iterations = 2000;
        parameters.f_rel = 0.05;
        parameters.x_rel = 0.1;
        parameters.time_penalty = 200.0;
        parameters.initial_stepsize_rel = 0.1;
        // parameters.inequality_constraint_tolerance = 0.1;
        parameters.inequality_constraint_tolerance = 0.2;

        mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension_,parameters);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
        // opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, 2*9.81f);
        opt.optimize();
        opt.getPolynomialOptimizationRef().getSegments(&segments);
        opt.getTrajectory((mav_trajectory_generation::Trajectory *)trajectory.get());
        }
    

    trajectory_mutex_.lock();
    traj_ptr_ = std::move(trajectory);
    endTime_ = traj_ptr_->getMaxTime();
    delay_t_ = (ros::Time::now()- t_i).toSec() + 0.0f;
    trajectory_mutex_.unlock();
    
    time_mutex_.lock();
    begin_time_ = ros::Time::now();
    time_mutex_.unlock();
    plot();

    // std::cout << "quitting Thread 1 "<< std::endl;


    // return checkTrajectoryFeasibility();

}

bool ETHSplineGenerator::generateTrajectory(const std::vector<std::vector<float>>& waypoints, float speed , const std::vector<float>& actual_speed_acc)
{
    if (gen_traj_thread_.joinable()) gen_traj_thread_.join();
    gen_traj_thread_ = std::thread(&ETHSplineGenerator::genTraj,this,waypoints,speed,actual_speed_acc);
    return true;
}

void ETHSplineGenerator::__plot(){
    // gen_traj_thread_.join();

    const float step = 0.1;
    trajectory_mutex_.lock();
    const float end_time = endTime_;
    trajectory_mutex_.unlock();
    nav_msgs::Path path;
    path.header.frame_id = "map";

    for (float t = 0.0f ; t < end_time ; t+= step){
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        trajectory_mutex_.lock();
        Eigen::VectorXd sample = traj_ptr_->evaluate(t,mav_trajectory_generation::derivative_order::POSITION);
        trajectory_mutex_.unlock();
        // ros::Duratio
        pose.pose.position.x = sample[0];
        pose.pose.position.y = sample[1];
        pose.pose.position.z = sample[2];
        path.poses.emplace_back(pose); 
    }
    static ros::NodeHandle nh ;
    static ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/debug/traj_ethz",1);
    path_pub.publish(path);
    std::cout << "plotting "<< std::endl;
}

void ETHSplineGenerator::plot(){
    if (plot_thread_.joinable())plot_thread_.join();
    plot_thread_ = std::thread(&ETHSplineGenerator::__plot,this);
}


bool ETHSplineGenerator::evaluateTrajectory(float t , std::array<std::array<float,3>,4>& refs){
    trajectory_mutex_.lock();

    if (traj_ptr_ == nullptr){
        std::cout << "no traj jet" << std::endl;
        trajectory_mutex_.unlock();
        return false;
    } 
    
    static auto last_refs = refs;
    t = t + delay_t_ + 0.2f;
    
    if (t > endTime_){
        if (t < endTime_ + 0.5){
            //set velocities and accels to 0.0f
            for (int i=0;i<refs.size();i++){
                refs[i][1]=0.0f;
                refs[i][2]=0.0f;
            }
        }
        // std::cerr<< "t:" <<t << " is > than "<< trajectory_.getMaxTime() <<std::endl;
        // t = (float) trajectory_.getMaxTime()-0.1;
    }
    else if (t<0) {
        t = 0.0f ;
        std::cerr<< "trajectory evaluated in t<0!!" <<std::endl;
    }
    else{

        int derivative_order;
        derivative_order = mav_trajectory_generation::derivative_order::POSITION;
        Eigen::VectorXd sample = traj_ptr_->evaluate(t, derivative_order);
        if (sample.size()>4) throw std::out_of_range("sample size is higher than 4\n");
        
        for (int i=0;i<sample.size();i++) refs[i][0]=sample[i];
        
        derivative_order = mav_trajectory_generation::derivative_order::VELOCITY;
        sample = traj_ptr_->evaluate(t, derivative_order);
        for (int i=0;i<sample.size();i++) refs[i][1]=sample[i];
        
        derivative_order = mav_trajectory_generation::derivative_order::ACCELERATION;
        sample = traj_ptr_->evaluate(t, derivative_order);
        for (int i=0;i<sample.size();i++)refs[i][2]=sample[i];

        #ifdef AUTOYAW 
        //refs[3][0] = -atan2f((double)refs[0][1],(double)refs[1][1])+M_PI/2.0f;
         refs[3][0] = 0;
        #endif
    
        
    }
    trajectory_mutex_.unlock();
    return true;

}

bool ETHSplineGenerator::checkTrajectoryFeasibility(){

    /*
    * Constraints:
    *  Acc       = [Acc_min, Acc_max]
    *  Vel       = [0 ,  Vel_max]
    *  Omega_pr  = [0 , Omega_pr_max]
    *  Omega_yaw = [0 , Omega_yaw_max] 
    *  acc_yaw   = [0 , acc_yaw_max]
    * */

    // Create input constraints.
   
   
   /*   TODO: CLEAN ALL THIS

    typedef mav_trajectory_generation::InputConstraintType ICT;
    mav_trajectory_generation::InputConstraints input_constraints; 

    input_constraints.addConstraint(ICT::kFMin,         constraints_.g_acc_min * 9.81);     // minimum acceleration in [m/s/s].
    input_constraints.addConstraint(ICT::kFMax,         constraints_.g_acc_max  * 9.81);    // maximum acceleration in [m/s/s].
    input_constraints.addConstraint(ICT::kVMax,         constraints_.vel_max );             // maximum velocity in [m/s].
    input_constraints.addConstraint(ICT::kOmegaXYMax,   constraints_.omega_pr_max );        // maximum roll/pitch rates in [rad/s].
    input_constraints.addConstraint(ICT::kOmegaZMax,    constraints_.omega_yaw_max);        // maximum yaw rates in [rad/s].
    input_constraints.addConstraint(ICT::kOmegaZDotMax, constraints_.acc_yaw_max);          // maximum yaw acceleration in [rad/s/s].

    // Create feasibility object of choice (FeasibilityAnalytic,FeasibilitySampling, FeasibilityRecursive).
    int feasibility_value = 0;
    
    // Check dynamic feasibility
    mav_trajectory_generation::FeasibilityAnalytic feasibility_check(input_constraints);
    feasibility_check.settings_.setMinSectionTimeS(0.01);
    int i = 0;

    // Check feasibility of each segment.
    for (auto segment:segments){
        mav_trajectory_generation::InputFeasibilityResult result = feasibility_check.checkInputFeasibility(segment);        
        if ((int)result != 0){
            std::cout << "The segment input(" << i << ") is " << getInputFeasibilityResultName(result) << "." << std::endl;
        } 
        feasibility_value += (int) result;
        i++;
    }

    // Check ground plane feasibility
    // mav_trajectory_generation::FeasibilityBase feasibility_check;
    
    // Create ground plane.
    Eigen::Vector3d point(0.0, 0.0, 0.5);
    Eigen::Vector3d normal(0.0, 0.0, 1.0);
    feasibility_check.half_plane_constraints_.emplace_back(point, normal);
    
    // Check feasibility.
    for (auto segment:segments){
        if(!feasibility_check.checkHalfPlaneFeasibility(segment)){
            std::cout << "The segment is in collision with the ground plane." << std::endl;
        feasibility_value++;
        }
    }
    
    if (feasibility_value == 0) return true;
    else{
        std::cout << "Trajectory generated is not feasible\n";
        return false;
    }
    */ return true ;


}
