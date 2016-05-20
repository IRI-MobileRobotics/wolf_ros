//
// Created by jvallve on 13/05/16.
//

#include "wolf_node.h"


int main(int argc, char **argv)
{
    std::cout << "\n=========== WOLF MAIN ===========\n\n";

    // Init ROS
    ros::init(argc, argv, "wolf_node");

    // Wolf ROS node
    WolfNode wolf_node(argv[0]);

    ros::Time last_solved = ros::Time::now();

    ros::Rate loopRate(10);

    while(ros::ok())
    {
        //execute pending callbacks
        ros::spinOnce();

        // solve every 1 seconds
//        if((ros::Time::now()- last_solved).toSec() > 0.1)
//        {
//            last_solved = ros::Time::now();
//            wolf_node.solve();
//        }
//        // broadcast transformations
//        else
            wolf_node.broadcastTf();

        //relax to fit output rate
        loopRate.sleep();
    }

    return 0;
}
