
#include "ros/ros.h"
#include "trajectory_publisher.hpp"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

nav_msgs::Odometry odom_msg;
void PoseCallback(const geometry_msgs::PoseStamped& msg){
    odom_msg.header = msg.header;
    odom_msg.header.frame_id = "map";
    odom_msg.pose.pose = msg.pose;
}

void TwistCallback(const geometry_msgs::TwistStamped& msg){
    odom_msg.header = msg.header;
    odom_msg.twist.twist.linear = msg.twist.linear;
}

void IMUCallback(const sensor_msgs::Imu& msg ){

    odom_msg.twist.twist.angular = msg.angular_velocity;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "trajectory_publisher_node");

    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("drone/odom",1);
    
    ros::Subscriber pose_sub = nh.subscribe("drone1/self_localization/pose",1,PoseCallback);
    ros::Subscriber speed_sub = nh.subscribe("drone1/self_localization/speed",1,TwistCallback);
    ros::Subscriber imu_sub = nh.subscribe("drone1/sensor_measurement/imu",1,IMUCallback);

    
    
    TrajectoryPublisher trajectory_publisher(Trajectory_type::eth_spline_non_linear);
    // TrajectoryPublisher trajectory_publisher(Trajectory_type::eth_spline_linear);
    trajectory_publisher.setup();

    ros::Rate rate(30);

    std::cout<< "TRAJECTORY GENERATOR RUNNING"<< std::endl;
    while(ros::ok())
    {
        //updating all the ros msgs
        
        ros::spinOnce();
        odom_pub.publish(odom_msg);
        ros::spinOnce();
        //running the localizer
        trajectory_publisher.run();

        rate.sleep();
    }

    return 0;
}
    


