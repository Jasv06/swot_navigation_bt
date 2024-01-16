/**
*       main.cpp
*
*       @date 01.01.2024
*       @author Joel Santos
*/

#include <iostream>
#include "swot_navigation/swot_robocup_navigation.h"

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv,  "Swot_Navigation_Node");

    // Create a AsyncSpinner with 4 threads
    ros::AsyncSpinner spinner(4);

    // Spin the spinner to process callbacks in multiple threads
    spinner.start();

    // Create an instance of the Navigation class
    // This instance calls the default constructor of the class
    Navigation newgoal;

    ros::waitForShutdown();   

    return 0;
}