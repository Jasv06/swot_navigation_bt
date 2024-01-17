/**
*       swot_robocup_navigation.h
*
*       @date 01.01.2023
*       @author Joel Santos
*/

#pragma once 

#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "nav_msgs/GetPlan.h"
#include "actionlib_msgs/GoalID.h"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm> 
#include "swot_msgs/SwotNavigation.h"
#include "swot_msgs/SwotPoseController.h"
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/action_node.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

enum class NavigationState { DRIVE_TO_FINISHED, HANDLE_NAV_RESULT, HANDLE_NAV_RESULT_FROM_RECOVERY };

class Navigation{

    private:
        //Nodehandle
        ros::NodeHandle nh;

        //ServiceServer for Communication to Taskplanning Algorithm
        ros::ServiceServer service_server;

        //Service Client for starting the Posecontroller
        ros::ServiceClient Pose_Controller_Client;

        //Actionlibclient for sending goals
        Client ac;

        //Variables
        bool executing;                             //Robot is currently executing a navigation tast to a given Goal
        bool near_by_WS;                            //logs if the Robot ist placed nearby a Workspace
        std::string Current_WS;                     //TODO: was "nan" before; is makePlan called before the first movement to estimate path lengths?
        bool CSV_RECOVERY;                          //Checks if "RECOVERY" is written correctly in CSV File
        bool CSV_FINISHED;                          //Checks if "RECOVERY" is written correctly in CSV File
        std::ifstream Workspace_Positions;
        std::string csv_file_path;
        std::vector<std::vector<std::string>> content;
        std::vector<std::vector<float>> pos;
        float time_to_navigate;                     //max time the client waits for a result 60 for slow, 45 for fast

        swot_msgs::SwotNavigation::Request request_;
        swot_msgs::SwotNavigation::Response response_;
        bool tree_status;
        NavigationState navState;

    public:

        //Class constructor
        Navigation();

        //Behavior tree function to register the bt nodes
        void registerNodes(BT::BehaviorTreeFactory& factory, Navigation& navigation);

        // Check if Action client works correctly
        void waitForMoveBase();

         // Read CSV File
        void readCSVFile();

        // Process CSV data
        void processCSVData();

        // Check CSV flags
        void checkCSVFlags();

        // Print status
        void printStatus();

        //This is the function that gets called everytime the service is called
        bool callback_service_navigation(swot_msgs::SwotNavigation::Request  &ws_message, swot_msgs::SwotNavigation::Response &res);

        //This function sends navigation goals
        void send_nav_goal(int CSV_Line);

        //This function handles the driving to multiple Controllerpositions and the Pushbacks
        void controll_Posecontroller(std::string workspace, bool pushback);

        //This function sends controller goals
        void send_controller_goal(int CSV_Line);

        //This function returns the right index values for the pos vector, for the content vektor you have to add 1 to get the correspondent line in the csv
        int search_line_in_csv(std::string workspace);

        //This function makes a plan
        void makePlan(bool setZero=false, std::string ws_goal="FINISHED");

        //set functions
        void set_response(std::string);
        void set_executing(bool);
        void set_near_by_WS(bool);
        void set_Current_WS(std::string);
        void set_CSV_RECOVERY(bool);
        void set_CSV_FINISHED(bool);
        void set_tree_status(bool);

        //get functions
        swot_msgs::SwotNavigation::Request get_request();
        Client get_ac();
        bool get_executing();
        bool get_near_by_WS();
        std::string get_Current_WS();
        bool get_CSV_RECOVERY();
        bool get_CSV_FINISHED();
        std::vector<std::vector<std::string>> get_content();
        bool get_tree_status();
        float get_time_to_navigate();
        NavigationState get_navState();

        // Reset the navigation state
        void resetNavigationState();

        // Tick function for handling the navigation result
        BT::NodeStatus tickHandleNavigationResult();

        // Tick function for handling the navigation result two
        BT::NodeStatus tickHandleNavigationTwo();

        // Tick function for handling the navigation result three
        BT::NodeStatus tickHandleNavigationThree();
        
        // Tick function for handling the navigation result Four
        BT::NodeStatus tickHandleNavigationFour();
};