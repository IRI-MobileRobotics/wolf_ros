//
// Created by jvallve on 13/05/16.
//

#include "wolf_node.h"


int main(int argc, char **argv)
{
    std::cout << "\n=========== WOLF MAIN ===========\n\n";

    // Init ROS
    ros::init(argc, argv, "wolf_node");

    // Wolf GPS ROS node
    WolfNode wolf_node();

    ros::Time last_solved = ros::Time::now();
    wolf_node.solve();

    ros::Rate loopRate(10);

    while(ros::ok())
    {
        //execute pending callbacks
        ros::spinOnce();

        // solve every 1 seconds
        if((ros::Time::now()- last_solved).toSec() > 1) //TODO: change to different threads for solving and getting odometry
        {
            last_solved = ros::Time::now();
            wolf_node.solve();
        }

        // get the odometry
        //TODO

        //relax to fit output rate
        loopRate.sleep();
    }

    return 0;
}
