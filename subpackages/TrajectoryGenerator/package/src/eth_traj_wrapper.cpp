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
    static ros::NodeHandle nh;
    static ros::Subscriber sub_pose = nh.subscribe("/drone1/self_localization/pose",1,&ETHSplineGenerator::poseCallback,this);
};


ETHSplineGenerator::~ETHSplineGenerator(){
    gen_traj_thread_.join();
};



void ETHSplineGenerator::genTraj(const std::vector<std::vector<float>>& waypoints, float speed , const std::vector<float>& actual_speed_acc)
{
    ros::Time t_i  = ros::Time::now(); 
    yaw_measured_ = waypoints[3][0];
    std::cout << "initial yaw" << waypoints[3][0]<<std::endl;    
    trajectory_mutex_.lock();
    float end_time = end_time_;
    float last_t_evaluated = last_t_evaluated_;
    trajectory_mutex_.unlock();

    // Waypoints[i][j] =>> i: (x,y,z,yaw) j:(waypoint_j)

    
    std::vector<mav_trajectory_generation::Vertex> vertices;
    int n_points = waypoints[0].size();
    const int n_points_added= 2;

    // Check if you must generate trajectory from a previous one or build one from scratch
    if (end_time -last_t_evaluated < 3*n_points_added*average_trajectory_generation_elapsed_time_ || end_time==0.0f ){
        std::cout << "GENERATING FROM SRATCH " <<std::endl; 
        // built it from scratch
        vertices = std::vector<mav_trajectory_generation::Vertex>(n_points,dimension_);
        vertices[0].addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(waypoints[0][0],waypoints[1][0],waypoints[2][0]));
        vertices[0].addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0,0,0));
        vertices[0].addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(0,0,0));   
    }else{
        // begin from previous trajectory references for the first 3 points
        std::cout << "GENERATING FROM PREVIOUS TRAJ " <<std::endl; 

        static std::array<std::array<float,3>,4> refs;
        const int n_points_added= 2;
        n_points = n_points + n_points_added-1; // -1 for the previous point that is not added
        vertices = std::vector<mav_trajectory_generation::Vertex>(n_points,dimension_);
        for (unsigned short int i = 0 ;i< n_points_added; i++){
            if (i==0) refs = last_sended_refs_;
            else {
                evaluateTrajectory(last_t_evaluated+i*2*average_trajectory_generation_elapsed_time_,refs);
            }
            vertices[i].addConstraint(mav_trajectory_generation::derivative_order::POSITION,    Eigen::Vector3d(refs[0][0],
                                                                                                                refs[1][0],
                                                                                                                refs[2][0]));
            vertices[i].addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,    Eigen::Vector3d(refs[0][1],
                                                                                                                refs[1][1],
                                                                                                                refs[2][1]));
            vertices[i].addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,Eigen::Vector3d(refs[0][2],
                                                                                                                refs[1][2],
                                                                                                                refs[2][2]));
        }
    }

    for (int i=1;i < n_points;i++){
        vertices[i].addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(waypoints[0][i],waypoints[1][i],waypoints[2][i]));
    }
    vertices[n_points-1].addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0,0,0));
    vertices[n_points-1].addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(0,0,0));

    //Calculate time slots

    double v_max = speed;
    auto segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices,v_max,a_max_);
    
    const int N = 10;
    // const int N = 6;
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
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max_);
        opt.optimize();
        opt.getPolynomialOptimizationRef().getSegments(&segments);
        opt.getTrajectory((mav_trajectory_generation::Trajectory *)trajectory.get());
        }

    const float time_alpha = 0.2;
    average_trajectory_generation_elapsed_time_ =  average_trajectory_generation_elapsed_time_*(1-time_alpha)+ time_alpha * (ros::Time::now()-t_i).toSec();
    
    next_trajectory_mutex_.lock();
    next_traj_ptr_ = std::move(trajectory);
    next_trajectory_mutex_.unlock();

    swapOldTrajectoryWithNewTrajectory();
    
    // return checkTrajectoryFeasibility();

}
void ETHSplineGenerator::poseCallback(const geometry_msgs::PoseStamped& msg){
    actual_pos_[0] = msg.pose.position.x; 
    actual_pos_[1] = msg.pose.position.y; 
    actual_pos_[2] = msg.pose.position.z; 
}

float locateDroneInTraj(Eigen::Vector3d actual_pos,const mav_trajectory_generation::Trajectory * trajectory ,float offset = 0.0f){
    const float step = 0.2;
    float dist = 0.0f;
    float min_dist = (actual_pos - trajectory->evaluate(0.0f)).norm(); 
    float delay_time = 0.0f;
    #define MAX_EVALUATION_TIME 5
    float evaluation_time = (trajectory->getMaxTime()<MAX_EVALUATION_TIME)?trajectory->getMaxTime():MAX_EVALUATION_TIME;

    for (float t = step; t < trajectory->getMaxTime();t+= step){
        dist = (actual_pos - trajectory->evaluate(t)).norm();
        if ( dist < min_dist){
            min_dist = dist;
            delay_time = t;
        }
    }
    return delay_time;
}



bool ETHSplineGenerator::generateTrajectory(const std::vector<std::vector<float>>& waypoints, float speed , const std::vector<float>& actual_speed_acc)
{
    trajectory_mutex_.lock();
    trajectory_generated_=false;
    trajectory_mutex_.unlock();

    if (gen_traj_thread_.joinable()) gen_traj_thread_.join();
    gen_traj_thread_ = std::thread(&ETHSplineGenerator::genTraj,this,waypoints,speed,actual_speed_acc);
    return true;
}

void ETHSplineGenerator::swapOldTrajectoryWithNewTrajectory(){
    
    // indent to ease lock understanding
    next_trajectory_mutex_.lock();
    float delay_t  = locateDroneInTraj(actual_pos_,next_traj_ptr_.get());
    next_trajectory_mutex_.unlock();

    trajectory_mutex_.lock();
        next_trajectory_mutex_.lock();
            traj_ptr_ = std::move(next_traj_ptr_);
            std::cout << "Trajectory swaped" << std::endl;
            next_traj_ptr_ = nullptr;
        next_trajectory_mutex_.unlock();
        delay_t_ = delay_t;
        end_time_ = traj_ptr_->getMaxTime();
            
        time_mutex_.lock();
            begin_time_ = ros::Time::now();
        time_mutex_.unlock();
        trajectory_generated_=true;
    trajectory_mutex_.unlock();

}
bool ETHSplineGenerator::checkTrajectorySwap(){
    std::cout << "CHECKING TRAJ" << std::endl;

    bool out = false;
    next_trajectory_mutex_.lock();
    if(next_traj_ptr_ == nullptr){;}
    else{
            out=true;   
            std::cout << "CHECK TRAJ SWAP = TRUE" << std::endl;
    }
    next_trajectory_mutex_.unlock();
    return out;
}

bool ETHSplineGenerator::evaluateTrajectory(float t , std::array<std::array<float,3>,4>& refs, bool for_plot){
    // std::cout << "EVALUATING TRAJ" << std::endl;
    

    
    // if (checkTrajectorySwap())swapOldTrajectoryWithNewTrajectory();
    trajectory_mutex_.lock();
    
    if (traj_ptr_ == nullptr){
        std::cout << "no traj jet" << std::endl;
        trajectory_mutex_.unlock();
        return false;
    } 
    
    // static auto last_refs = refs;
    t = t + delay_t_;
    last_t_evaluated_ = t;
    
    if (t > end_time_){
        if (t < end_time_ + 0.5){
            traj_evaluation_finished_ = true;
            //set velocities and accels to 0.0f
            for (int i=0;i<refs.size();i++){
                refs[i][1]=0.0f;
                refs[i][2]=0.0f;
            }
        }       
    }
    else if (t<0) {
        t = 0.0f ;
        std::cerr<< "trajectory evaluated in t<0!!" <<std::endl;
    }
    else{
        const float alpha = 1;
        int derivative_order;
        derivative_order = mav_trajectory_generation::derivative_order::POSITION;
        Eigen::VectorXd sample = traj_ptr_->evaluate(t, derivative_order);
        if (sample.size()>4) throw std::out_of_range("sample size is higher than 4\n");
        
        for (int i=0;i<sample.size();i++) refs[i][0]= sample[i];
        
        derivative_order = mav_trajectory_generation::derivative_order::VELOCITY;
        sample = traj_ptr_->evaluate(t, derivative_order);
        for (int i=0;i<sample.size();i++) refs[i][1]= sample[i];
        
        derivative_order = mav_trajectory_generation::derivative_order::ACCELERATION;
        sample = traj_ptr_->evaluate(t, derivative_order);
        for (int i=0;i<sample.size();i++)refs[i][2]=  sample[i];

        #ifdef AUTOYAW 
            // refs[3][0] = -atan2f((double)refs[0][1],(double)refs[1][1])+M_PI/2.0f;
            refs[3][0] = 0;
        #endif
        
    }
    if (!for_plot) last_sended_refs_= refs;
    trajectory_mutex_.unlock();
    return true;

}

bool ETHSplineGenerator::checkTrajectoryFeasibility(){
    
    //TODO USE THIS

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
    */ 
   return true ;


}
