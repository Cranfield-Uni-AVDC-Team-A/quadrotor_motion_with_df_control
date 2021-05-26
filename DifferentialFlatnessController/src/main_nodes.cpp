#include "ros/ros.h"
#include "PD_controller.hpp"

#include <dynamic_reconfigure/server.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv,"pd_controller_node");

    PD_controller my_controller;
    my_controller.setUp();


    #ifdef DYNAMIC_TUNING

        //dynamic Reconfigure
        dynamic_reconfigure::Server<pd_controller::ControllerConfig> server;
        dynamic_reconfigure::Server<pd_controller::ControllerConfig>::CallbackType f;

        f = boost::bind(&PD_controller::parametersCallback, &my_controller,_1, _2);
        server.setCallback(f);

    #endif

    ros::Rate rate(100);
    while(ros::ok())
    {
        //updating all the ros msgs
        ros::spinOnce();
        //running the localizer
        my_controller.run();
        rate.sleep();
    }

    return 0;
}
    