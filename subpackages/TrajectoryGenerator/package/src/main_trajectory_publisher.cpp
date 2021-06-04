
#include "ros/ros.h"
#include "trajectory_publisher.hpp"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "trajectory_publisher_node");

    
    TrajectoryPublisher trajectory_publisher(Trajectory_type::eth_spline_non_linear);
    // TrajectoryPublisher trajectory_publisher(Trajectory_type::eth_spline_linear);
    trajectory_publisher.setup();

    ros::Rate rate(30);

    std::cout<< "TRAJECTORY GENERATOR RUNNING"<< std::endl;
    while(ros::ok())
    {
        //updating all the ros msgs
        
        ros::spinOnce();
        //running the localizer
        trajectory_publisher.run();

        rate.sleep();
    }

    return 0;
}
    


