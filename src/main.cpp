/**
*       main.cpp
*
*       @date 18.12.2023
*       @author Joel Santos
*/

#include <iostream>
#include "swot_navigation/navigation.h"

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "swot_navigation");

    // Create an instance of the Navigation class
    // This instance calls the default constructor of the class
    Navigation navigation;

    // Initialize the navigation class and its ROS services
    navigation.initialize();
    // Create a MultiThreadedSpinner with 4 threads
    ros::MultiThreadedSpinner spinner(4);
    // Spin the spinner to process callbacks in multiple threads
    spinner.spin();

    return 0;
}

