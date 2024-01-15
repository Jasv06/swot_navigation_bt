#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "actionlib_msgs/GoalID.h"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include "swot_msgs/SwotNavigation.h"
#include "swot_msgs/SwotPoseController.h"
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

bool global_print_debug = true;

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
        bool executing = false; //Robot is currently executing a navigation tast to a given Goal
        bool near_by_WS = false; //logs if the Robot ist placed nearby a Workspace
        std:: string Current_WS = "nan";
        bool CSV_RECOVERY = false; //Checks if "RECOVERY" is written correctly in CSV File
        bool CSV_FINISHED = false; //Checks if "RECOVERY" is written correctly in CSV File
        std::ifstream Workspace_Positions;
        std::string csv_file_path;
        std::vector<std::vector<std::string>> content;
        std::vector<std::vector<float>> pos;
        int ID_CSV = 0; //index to Navigate through the lines of the CSV file
        float time_to_navigate = 60.0;// max time the client waits for a result 60 for slow, 45 for fast

    public:

        Navigation() : ac("move_base", true){
            //Check if Action client works correctly
            while(!ac.waitForServer(ros::Duration(5.0))){
                if(global_print_debug == true){
                    ROS_INFO("Waiting for the move_base action server to come up");
                }
            }

            //define service_server
            service_server = nh.advertiseService("swot_next_destination", &Navigation::callback_service_navigation, this);
            
            //define Posecontrollerclient
            Pose_Controller_Client = nh.serviceClient<swot_msgs::SwotPoseController>("PoseControllerServer");
            //Read CSV File
            csv_file_path = ros::package::getPath("swot_navigation")+"/src/WS_Positions.csv";
            if(global_print_debug == true){
                ROS_INFO_STREAM("Filepath: " << csv_file_path);
            }
            std::vector<std::string> row;
            std::string line, word;
            std::fstream file (csv_file_path, std::ios::in);
            if(file.is_open()){
                while(getline(file, line)){
                    row.clear();
                    std::stringstream str(line);
                    while(getline(str, word, ',')){
                        row.push_back(word);
                    }
                    content.push_back(row);
                }
            }
            else {
                if(global_print_debug == true){
                    ROS_ERROR("Could not open CSV File!!!");
                }
                ros::shutdown();
            }
            for (int i = 1; i<content.size(); i++){ //anzahl der zeilen
                std::vector<float> dummy;
                dummy.clear();
                for(int j = 1; j<content[i].size(); j++){
                    dummy.push_back(atof(content[i][j].c_str()));
                }
                pos.push_back(dummy);
            }
            for(int i = 1; i<content.size(); i++){
                if(content[i][0].compare("RECOVERY") == 0){
                    CSV_RECOVERY = true;
                }
                if(content[i][0].compare("FINISHED") == 0){
                    CSV_FINISHED = true;
                }
            }
            if((CSV_RECOVERY == false) || (CSV_FINISHED == false)){
                if(global_print_debug == true){
                    ROS_ERROR("Fehlerhafte CSV");
                }
                ros::shutdown();
            }
            if(global_print_debug == true){
                ROS_INFO("Navigation ready for Competition!!!");
            }
        }

        bool callback_service_navigation(swot_msgs::SwotNavigation::Request  &ws_message, swot_msgs::SwotNavigation::Response &res){
            std::string GOAL_WS_ID = (ws_message.destination);
            bool found = false;
            for(int i = 1; i<content.size(); i++){ //Check if the destinated Goal is contained in the CSV File
                if(content[i][0].compare(GOAL_WS_ID) == 0){
                    found = true;
                }
            }
            if(found == false){//If the destinated Goal isn't containted in the CSV File Quit the Service with Response "Failed"
                if(global_print_debug == true){
                    ROS_WARN_STREAM("The Goal "<< GOAL_WS_ID << " doesn't exist in the CSV_File. Service Failed");
                }
                res.status = "FAILED";
                return true;
            }
            if(global_print_debug == true){
                ROS_INFO_STREAM("Got new Goal! The next destination is: " << GOAL_WS_ID);
            }
            if(near_by_WS == true){
                if(global_print_debug == true){
                    ROS_INFO("Doing a Pushback to the previous NAV GOAL of the Robot in order to Navigate Normally!");
                }
                controll_Posecontroller(Current_WS, true);
                if(global_print_debug == true){
                    ROS_INFO("Pushback Completed");
                }
                near_by_WS = false;
            }
            if(GOAL_WS_ID.compare("FINISHED")==0){
                if(global_print_debug == true){
                    ROS_INFO("Robot is driving to the Endpose!");
                }
                send_nav_goal(search_line_in_csv("FINISHED"));
                if(!ac.waitForResult(ros::Duration(time_to_navigate))){//there is no result of the Navigation
                    if(global_print_debug == true){
                        ROS_WARN("Navigation Task to the Endpose failed for the first Time due to Timeout!");
                        ROS_INFO("Sending the Endpose NAV_GOAL again!");
                    }
                    send_nav_goal(search_line_in_csv("FINISHED"));
                    if(!ac.waitForResult(ros::Duration(time_to_navigate))){
                        if(global_print_debug == true){
                            ROS_WARN("Navigation Failed for the second Time due to Timeout!");
                            ROS_ERROR("Cancel the Navigation Task!!");
                        }
                        ac.cancelGoal();
                        res.status = "FAILED";
                        return true;
                    }
                    else{//There is a Result for the Second Try to drive to the FINISHED Pose
                        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                            if(global_print_debug == true){
                                ROS_INFO("Robot reached the Finished Pose!");
                            }
                            res.status = "FINISHED";
                            near_by_WS = false;
                            Current_WS = "FINISHED";
                            return true;
                        }
                        if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                            if(global_print_debug == true){
                                ROS_INFO("Robot Can't reach the Finished Pose due to Obstacles, Mislocalization or sensor errors!");
                                ROS_INFO("Drive the Robot to Recovery Pose!");
                            }
                            send_nav_goal(search_line_in_csv("RECOVERY"));
                            if(!ac.waitForResult(ros::Duration(time_to_navigate))){//Wait while the Robot is driving to Recovery Pose
                                if(global_print_debug == true){
                                    ROS_WARN("Robot can't Reach Recovery Pose and NAV Goal Timed out!");
                                    ROS_ERROR("Cancel Goal and service!");
                                }
                                ac.cancelGoal();
                                res.status = "FAILED";
                                return true;
                            }
                            else{
                                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){//Robot reached the Recovery Pose successfully
                                    if(global_print_debug == true){
                                        ROS_INFO("Robot reached the Recovery Pose!");
                                        ROS_INFO("Sending the NAV GOAL to the Finished Pose again!");
                                    }
                                    send_nav_goal(search_line_in_csv("FINISHED"));
                                    if(!ac.waitForResult(ros::Duration(time_to_navigate))){//Wait while the Robot is driving From the Recovery Pose to Finished pose
                                        if(global_print_debug == true){
                                            ROS_WARN("Robot can't reach the FINISHED Pose from the Recovery Pose due to timeout!");
                                            ROS_INFO("Cancel NAV GOAL and Service!");
                                        }
                                        ac.cancelGoal();
                                        res.status = "FAILED";
                                        return true;    
                                    }
                                    else{
                                        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                            if(global_print_debug == true){
                                                ROS_INFO("Robot reached the Finished Pose!");
                                            }
                                            res.status = "FINISHED";
                                            near_by_WS = false;
                                            Current_WS = "FINISHED";
                                            return true;
                                        }
                                        if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                                            if(global_print_debug == true){
                                                ROS_WARN("Robot can't reach the FINISHED Pose from the Recovery Pose");
                                                ROS_ERROR("Cancel Goal and service!");
                                            }
                                            ac.cancelGoal();
                                            res.status = "FAILED";
                                            return true;
                                        }

                                    }
                                }
                                if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                                    if(global_print_debug == true){
                                        ROS_WARN("Robot can't reach the Recovery Pose!");
                                        ROS_ERROR("Cancelling NAV GOAL and Service!");
                                    }
                                    ac.cancelGoal();
                                    res.status = "FAILED";
                                    return true;
                                }
                            }
                        }
                    }
                }
                else{
                    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){//FINISH Goal pose reached successfully
                        if(global_print_debug == true){
                            ROS_INFO("Robot Reached the Endpose!");
                        }
                        res.status = "FINISHED";
                        near_by_WS = false;
                        Current_WS = "FINISHED";
                        return true;
                    }
                    if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){//Aborted Navigation task to finished Pose
                        if(global_print_debug == true){
                            ROS_WARN("Robot didn't Reach the Endpose!");
                            ROS_INFO("Sending the Endpose NAV_GOAL again!");
                        }
                        send_nav_goal(search_line_in_csv("FINISHED"));
                        if(!ac.waitForResult(ros::Duration(time_to_navigate))){//Wait while the robot drives to the FINISHED Pose for the second time
                            if(global_print_debug == true){
                                ROS_WARN("Second Navigation Task Failed due to Timeout");
                                ROS_INFO("Driving to the Recovery Pose!");
                            }
                            send_nav_goal(search_line_in_csv("RECOVERY"));
                            if(!ac.waitForResult(ros::Duration(time_to_navigate))){
                                if(global_print_debug == true){
                                    ROS_WARN("Can't reach Recovery Pose!");
                                    ROS_ERROR("Cancel NAV GOAL and Service!");
                                }
                                ac.cancelGoal();
                                res.status = "FAILED";
                                return true;
                            }
                            else{
                                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){//FINISH Goal pose reached successfully
                                    if(global_print_debug == true){
                                        ROS_INFO("Robot Reached the Recovery Pose!");
                                        ROS_INFO("Drive to the FINISHED Pose!");
                                    }
                                    send_nav_goal(search_line_in_csv("FINISHED"));
                                    if(!ac.waitForResult(ros::Duration(time_to_navigate))){
                                        if(global_print_debug == true){
                                            ROS_WARN("Navigation Task: Driving vom ReconveryPose to FINISHED Pose Timed out");
                                            ROS_ERROR("Cancel NAV GOAL and Service!");
                                        }
                                        ac.cancelGoal();
                                        res.status = "FAILED";
                                        return true;
                                    }
                                    else{
                                        if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                                            if(global_print_debug == true){
                                                ROS_WARN("Can't reach the desired NAV GOAL from the Recovery Pose due to sensor errors, mislocalization etc.!");
                                                ROS_ERROR("Cancel NAV GOAL and Service!");
                                            }
                                            ac.cancelGoal();
                                            res.status = "FAILED";
                                            return true;
                                        }
                                        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                            if(global_print_debug == true){
                                                ROS_INFO("Robot Successfully Drive to the Finished Pose!");
                                            }
                                            res.status = "FINISHED";
                                            near_by_WS = false;
                                            Current_WS = "FINISHED";
                                            return true;
                                        }
                                    }
                                }
                                if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                                    if(global_print_debug == true){
                                        ROS_WARN("Robot can't reach Recovery Pose!");
                                        ROS_ERROR("Cancel NAV GOAL and Service!");
                                    }
                                    ac.cancelGoal();
                                    res.status = "FAILED";
                                    return true;
                                }

                            }
                            if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                                if(global_print_debug == true){
                                    ROS_WARN("Can't reach the Recovery Pose!");
                                    ROS_ERROR("Cancel NAV GOAL and Service!");
                                }
                                ac.cancelGoal();
                                res.status = "FAILED";
                                return true;
                            }

                        }
                        else{
                            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                if(global_print_debug == true){
                                    ROS_INFO("Robot Reached the FINISHED Pose!");
                                }
                                res.status = "FINISHED";
                                near_by_WS = false;
                                Current_WS = "FINISHED";
                                return true;
                            }
                            if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                                if(global_print_debug == true){
                                    ROS_WARN("Robot aborted the second Navigation Task to the FINISHED POSE!");
                                    ROS_INFO("Driving to the RECOVERY POSE");
                                }
                                send_nav_goal(search_line_in_csv("RECOVERY"));
                                if(!ac.waitForResult(ros::Duration(time_to_navigate))){
                                    if(global_print_debug == true){
                                        ROS_WARN("Navigation task to Recovery Pose timed out!!");
                                        ROS_ERROR("Cancel NAV GOAL and Service!!");
                                    }
                                    ac.cancelGoal();
                                    res.status = "FAILED";
                                    return true;
                                }
                                else{
                                    if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                                        if(global_print_debug == true){
                                            ROS_INFO("Robot can't reach the Recovery Pose!");
                                            ROS_ERROR("Cancel NAV GOAL and Service!!");
                                        }
                                        ac.cancelGoal();
                                        res.status = "FAILED";
                                        return true;
                                    }
                                    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                        if(global_print_debug == true){
                                            ROS_INFO("Robot reached the Recovery Pose!");
                                            ROS_INFO("Driving to the FINISHED Position from the Recovery Position!");
                                        }
                                        send_nav_goal(search_line_in_csv("FINISHED"));
                                        if(!ac.waitForResult(ros::Duration(time_to_navigate))){
                                            if(global_print_debug == true){
                                                ROS_WARN("Robot can't reach the FINISHED Pose from the Recovery Pose in Time");
                                                ROS_ERROR("Cancel NAV GOAL and Service!!");
                                            }
                                            ac.cancelGoal();
                                            res.status = "FAILED";
                                            return true;
                                        }
                                        else{
                                            if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                                                if(global_print_debug == true){
                                                    ROS_WARN("Navigation Task: Driving from Recovery Pose to Finished Pose aborted");
                                                    ROS_ERROR("Cancel NAV GOAL and Service!!");
                                                }
                                                ac.cancelGoal();
                                                res.status = "FAILED";
                                                return true;
                                            }
                                            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                                if(global_print_debug == true){
                                                    ROS_INFO("Robot reached the FINISHED Pose!");
                                                }
                                                res.status = "FINISHED";
                                                near_by_WS = false;
                                                Current_WS = "FINISHED";
                                                return true;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            else if(GOAL_WS_ID.compare("RECOVERY")==0){
                if(global_print_debug == true){
                    ROS_INFO("Robot is driving to a Recoverypose!");
                }
                send_nav_goal(search_line_in_csv("RECOVERY"));
                if(!ac.waitForResult(ros::Duration(time_to_navigate))){//there is no result of the Navigation
                    if(global_print_debug == true){
                        ROS_WARN("Navigation Task to the Recoverypose failed due to Timeout!");
                    }
                    ac.cancelGoal();
                    res.status = "FAILED";
                    return true;
                }
                else{
                    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){//Second Nav GOAL Reached successfully
                        if(global_print_debug == true){
                            ROS_INFO("Robot Reached the Recovery Position!");
                        }
                        res.status = "FINISHED";
                        near_by_WS = false;
                        Current_WS = "RECOVERY";
                        return true;
                    }
                    if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                        if(global_print_debug == true){
                            ROS_WARN("Robot can't reach the Recovery Position!!");
                            ROS_ERROR("Cancel NAV_GOAL and Service!!");
                        }
                        ac.cancelGoal();
                        res.status = "FAILED";
                        return true;
                    }
                }
            }
            else{
                if(global_print_debug == true){
                    ROS_INFO_STREAM("Robot is driving to: " << GOAL_WS_ID);
                }
                send_nav_goal(search_line_in_csv(GOAL_WS_ID));
                if(!ac.waitForResult(ros::Duration(time_to_navigate))){//First Navgoal timed out 
                    if(global_print_debug == true){
                        ROS_WARN_STREAM("Navigation Task to " << GOAL_WS_ID << " failed! due to Timeout!!");
                        ROS_INFO_STREAM("Sending the destination " << GOAL_WS_ID << " for the second time!!");
                    }
                    send_nav_goal(search_line_in_csv(GOAL_WS_ID));
                    if(!ac.waitForResult(ros::Duration(time_to_navigate))){ //Second Navgoal timed out 
                        if(global_print_debug == true){
                            ROS_WARN_STREAM("Navigation Task to " << GOAL_WS_ID << " failed for the second time due to Timeout!!");
                        }
                        near_by_WS = false;
                        Current_WS = "nan";
                        ac.cancelGoal();
                        res.status = "FAILED";
                        return true;
                    }
                    else{//Result for the Second Nav goal
                        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){//Second Nav GOAL Reached successfully
                            if(global_print_debug == true){
                                ROS_INFO("Robot Reached the NAV_GOAL for the desired Workspace");
                                ROS_INFO("Trigger Pose Controller!");
                            }
                            controll_Posecontroller(GOAL_WS_ID, false);
                            if(global_print_debug == true){
                                ROS_INFO_STREAM("Robot near by " << GOAL_WS_ID);
                            }
                            near_by_WS = true;
                            Current_WS = GOAL_WS_ID;
                            res.status = "FINISHED";
                            return true;
                        }
                        if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){//Didn't reach second Nav GOAL
                            if(global_print_debug == true){
                                ROS_WARN("Robot can't reach the desired NAV_Goal due to some Obstacles or mislocaization or Sensor errors etc.");
                                ROS_INFO("Robot drives to Recovery Position");
                            }
                            send_nav_goal(search_line_in_csv("RECOVERY")); //Send NAV GOAL to Recovery Pose
                            if(!ac.waitForResult(ros::Duration(time_to_navigate))){ //Drive to the RECOVERY Pose timed out
                                if(global_print_debug == true){
                                    ROS_WARN("Failed to Drive to Recoveryposition in Time!");
                                    ROS_ERROR("Cancel Navigation");
                                }
                                ac.cancelGoal();
                                res.status = "FAILED";
                                near_by_WS = false;
                                Current_WS = "nan";
                                return true;
                            }
                            else{// Navigation result for driving to the RECOVERY Pose
                                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){ //successfully reached the Recoverypose
                                    if(global_print_debug == true){
                                        ROS_INFO("Robot reached the Recovery Pose successfully!");
                                        ROS_INFO("Send the initial Goal again!");
                                    }
                                    send_nav_goal(search_line_in_csv(GOAL_WS_ID)); //Sending the initial Goal again
                                    if(!ac.waitForResult(ros::Duration(time_to_navigate))){//Wait for the Navigation Result for the inital NAV_GOAL
                                        if(global_print_debug == true){
                                            ROS_WARN("Robot can't Reach the initial goal from the Recovery Pose due to Timeout");
                                        }
                                        ac.cancelGoal();
                                        res.status = "FAILED";
                                        near_by_WS = false;
                                        Current_WS = "nan";
                                        return true;
                                    }
                                    else {
                                        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                            if(global_print_debug == true){
                                                ROS_INFO("Robot reached the Nav GOAL Desired Service Area from the Recovery Pose!");
                                            }
                                            controll_Posecontroller(GOAL_WS_ID, false);
                                            if(global_print_debug == true){
                                                ROS_INFO_STREAM("Robot near by " << GOAL_WS_ID);
                                            }
                                            near_by_WS = true; 
                                            Current_WS = GOAL_WS_ID;
                                            res.status = "FINISHED";
                                            return true;
                                        }
                                        if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                                            if(global_print_debug == true){
                                                ROS_WARN("Robot can't reach the desired NAV_GOAL from the Recovery Pose");
                                            }
                                            ac.cancelGoal();
                                            res.status = "FAILED";
                                            near_by_WS = false;
                                            Current_WS = "nan";
                                            return true;
                                        }
                                    }
                                }
                                if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                                    if(global_print_debug == true){
                                        ROS_WARN("Robot can't reach Recovery Pose!");
                                        ROS_INFO("Cancelling NAV GOAL!");
                                    }
                                    ac.cancelGoal();
                                    res.status = "FAILED";
                                    near_by_WS = false;
                                    Current_WS = "nan";
                                    return true;
                                }
                            }

                        }
                    } 
                }
                else{//there is a result of the Navigation
                    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                        if(global_print_debug == true){
                            ROS_INFO_STREAM("Robot Reached " << GOAL_WS_ID);
                        }
                        controll_Posecontroller(GOAL_WS_ID, false);
                        if(global_print_debug == true){
                            ROS_INFO_STREAM("Robot near by " << GOAL_WS_ID);
                        }
                        near_by_WS = true;
                        Current_WS = GOAL_WS_ID;
                        res.status = "FINISHED";
                        return true;

                    }
                    if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                        if(global_print_debug == true){
                            ROS_WARN_STREAM("Navigation Task: Driving to " << GOAL_WS_ID << " was aborded due to mislocalization or sensor issues");
                            ROS_INFO_STREAM("Sending the Navigation Goal to " << GOAL_WS_ID << " once again!!");
                        }
                        send_nav_goal(search_line_in_csv(GOAL_WS_ID));
                        if(!ac.waitForResult(ros::Duration(time_to_navigate))){
                            if(global_print_debug == true){
                                ROS_WARN_STREAM("Navigation TASK to " << GOAL_WS_ID << " timed out!");
                                ROS_INFO("Driving to RecoveryPose!");
                            }
                            send_nav_goal(search_line_in_csv("RECOVERY"));
                            if(!ac.waitForResult(ros::Duration(time_to_navigate))){
                                if(global_print_debug == true){
                                    ROS_WARN("Can't Reach Recovery Pose due to time out");
                                    ROS_ERROR("Cancel Navigation GOAL and Service!!");
                                }
                                near_by_WS = false;
                                Current_WS = "nan";
                                ac.cancelGoal();
                                res.status = "FAILED";
                                return true;
                            }
                            else{
                                if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                                    if(global_print_debug == true){
                                        ROS_WARN("Can't Reach Recovery Pose due to obstacles, mislocalization, sonsorerrors etc.");
                                        ROS_ERROR("Cancel Navigation GOAL and Service!!");
                                    }
                                    near_by_WS = false;
                                    Current_WS = "nan";
                                    ac.cancelGoal();
                                    res.status = "FAILED";
                                    return true;
                                }
                                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                    if(global_print_debug == true){
                                        ROS_INFO("Robot reached the Recovery Pose!");
                                        ROS_INFO_STREAM("Sending the Navigation GOAL to " << GOAL_WS_ID << " once again!");
                                    }
                                    send_nav_goal(search_line_in_csv(GOAL_WS_ID));
                                    if(!ac.waitForResult(ros::Duration(time_to_navigate))){
                                        if(global_print_debug == true){
                                            ROS_WARN_STREAM("Second Navigation GOAL to " << GOAL_WS_ID << " also timed out!");
                                            ROS_ERROR("Cancelling Nav GOAL and Service!");
                                        }
                                        ac.cancelGoal();
                                        near_by_WS = false;
                                        Current_WS = "nan";
                                        res.status = "FAILED";
                                        return true;
                                    }
                                    else{
                                        if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                                            if(global_print_debug == true){
                                                ROS_WARN_STREAM("Second Navigation GOAL to " << GOAL_WS_ID << " not reachable due to mislocalization, sensor errors etc. ");
                                                ROS_ERROR("Cancelling Nav GOAL and Service!");
                                            }
                                            ac.cancelGoal();
                                            near_by_WS = false;
                                            Current_WS = "nan";
                                            res.status = "FAILED";
                                            return true;
                                        }
                                        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                            if(global_print_debug == true){
                                                ROS_INFO_STREAM("Reached " << GOAL_WS_ID);
                                            }
                                            controll_Posecontroller(GOAL_WS_ID, false);
                                            if(global_print_debug == true){
                                                ROS_INFO_STREAM("Robot near by " << GOAL_WS_ID);
                                            }
                                            near_by_WS = true;
                                            Current_WS = GOAL_WS_ID;
                                            res.status = "FINISHED";
                                            return true;
                                        }

                                    }

                                }
                            }
                        }
                        else{
                            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                if(global_print_debug == true){
                                    ROS_INFO_STREAM("Reached " << GOAL_WS_ID);
                                }
                                controll_Posecontroller(GOAL_WS_ID, false);
                                if(global_print_debug == true){
                                    ROS_INFO_STREAM("Robot near by " << GOAL_WS_ID);
                                }
                                near_by_WS = true;
                                Current_WS = GOAL_WS_ID;
                                res.status = "FINISHED";
                                return true;
                            }
                            if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                                if(global_print_debug == true){
                                    ROS_WARN_STREAM("Robot can't reach "<< GOAL_WS_ID << " due to mislocalization, obstacles, sensor errors etc!");
                                    ROS_INFO("Sending the Robot to the Recovery Pose!");
                                }
                                send_nav_goal(search_line_in_csv("RECOVERY"));
                                if(!ac.waitForResult(ros::Duration(time_to_navigate))){
                                    if(global_print_debug == true){
                                        ROS_WARN("Robot can't Reach the Recovery Pose in Time!");
                                        ROS_ERROR("Cancelling Nav GOAL and Service!");
                                    }
                                    ac.cancelGoal();
                                    near_by_WS = false;
                                    Current_WS = "nan";
                                    res.status = "FAILED";
                                    return true;
                                }
                                else{
                                    if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                                        if(global_print_debug == true){
                                            ROS_WARN("Robot can't Reach the Recovery Pose due to Mislocalization, obstacle, sensor errors etc!");
                                            ROS_ERROR("Cancelling Nav GOAL and Service!");
                                        }
                                        ac.cancelGoal();
                                        near_by_WS = false;
                                        Current_WS = "nan";
                                        res.status = "FAILED";
                                        return true;
                                    }
                                    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                        if(global_print_debug == true){
                                            ROS_INFO("Robot reached the Recovery Pose!");
                                            ROS_INFO_STREAM("Drivng the Robot to " << GOAL_WS_ID);
                                        }
                                        send_nav_goal(search_line_in_csv(GOAL_WS_ID));
                                        if(!ac.waitForResult(ros::Duration(time_to_navigate))){
                                            if(global_print_debug == true){
                                                ROS_WARN_STREAM("Robot can't reach " << GOAL_WS_ID << " for the second time due to Timeout");
                                                ROS_ERROR("Cancelling Nav GOAL and Service!");
                                            }
                                            ac.cancelGoal();
                                            near_by_WS = false;
                                            Current_WS = "nan";
                                            res.status = "FAILED";
                                            return true;
                                        }
                                        else{
                                            if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                                                if(global_print_debug == true){
                                                    ROS_WARN_STREAM("Robot can't Reach " << GOAL_WS_ID << "due to Mislocalization, obstacle, sensor errors etc!");
                                                    ROS_ERROR("Cancelling Nav GOAL and Service!");
                                                }
                                                ac.cancelGoal();
                                                near_by_WS = false;
                                                Current_WS = "nan";
                                                res.status = "FAILED";
                                                return true;
                                            }
                                            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                                if(global_print_debug == true){
                                                    ROS_INFO_STREAM("Reached " << GOAL_WS_ID);
                                                }
                                                controll_Posecontroller(GOAL_WS_ID, false);
                                                ROS_INFO_STREAM("Robot near by " << GOAL_WS_ID);
                                                near_by_WS = true;
                                                Current_WS = GOAL_WS_ID;
                                                res.status = "FINISHED";
                                                return true;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            return(0);
        }
        void send_nav_goal(int CSV_Line){
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.seq = 1;
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.pose.position.x = pos[CSV_Line][0];
            goal.target_pose.pose.position.y = pos[CSV_Line][1];
            goal.target_pose.pose.position.z = pos[CSV_Line][2];
            goal.target_pose.pose.orientation.x = pos[CSV_Line][3];
            goal.target_pose.pose.orientation.y = pos[CSV_Line][4];
            goal.target_pose.pose.orientation.z = pos[CSV_Line][5];
            goal.target_pose.pose.orientation.w = pos[CSV_Line][6];
            ac.sendGoal(goal);
        }

        void controll_Posecontroller(std::string workspace, bool pushback){//this function handles the driving to multiple Controllerpositions and the Pushbacks
            if(workspace.compare("RECOVERY") == 0){
                if(global_print_debug == true){
                    ROS_INFO("No Controller Goal for Driving to Recovery!");
                }
                return;
            }
            if(workspace.compare("FINISHED") == 0){
                if(global_print_debug == true){
                    ROS_INFO("No Controller Goal for Driving to Finished!");
                }
                return;
            }
            int number_of_controller_positions = 0;
            for(int i = 1; i<content.size();i++){
                if(content[i][0].find(workspace + "_controller") != std::string::npos){
                    number_of_controller_positions++;
                }
            }
            if(global_print_debug == true){
                ROS_INFO_STREAM("Number of controller Positions: " << number_of_controller_positions);
            }
            if(pushback == false){
                for(int j = 1; j <= number_of_controller_positions; j++){
                    std::string string_to_send = "nan";
                    string_to_send = workspace + "_controller" + std::to_string(j);
                    send_controller_goal(search_line_in_csv(string_to_send));
                }
            }
            else{
                if(number_of_controller_positions == 1){
                    send_controller_goal(search_line_in_csv(workspace));
                }
                else{
                    for (int j = number_of_controller_positions - 1; j >= 0; j--){
                        if(j!=0){
                            std::string string_to_send = "nan";
                            string_to_send = workspace + "_controller" + std::to_string(j);
                            send_controller_goal(search_line_in_csv(string_to_send));
                        }
                        else{
                            std::string string_to_send = "nan";
                            string_to_send = workspace;
                            send_controller_goal(search_line_in_csv(workspace));
                        }
                    }
                }
            }
        }

        void send_controller_goal(int CSV_Line){
            if(global_print_debug == true){
                ROS_INFO_STREAM("Robot is driving with Pose Controller to " << content[CSV_Line+1][0]);
            }
            swot_msgs::SwotPoseController srv;
            srv.request.controllerGoal.position.x = pos[CSV_Line][0];
            srv.request.controllerGoal.position.y = pos[CSV_Line][1];
            srv.request.controllerGoal.position.z = pos[CSV_Line][2];
            srv.request.controllerGoal.orientation.x = pos[CSV_Line][3];
            srv.request.controllerGoal.orientation.y = pos[CSV_Line][4];
            srv.request.controllerGoal.orientation.z = pos[CSV_Line][5];
            srv.request.controllerGoal.orientation.w = pos[CSV_Line][6];
            if(Pose_Controller_Client.call(srv)){
                if(!srv.response.status.compare("FINISHED")){
                    if(global_print_debug == true){
                        ROS_INFO(" ");
                    }
                }
            }
        }

        int search_line_in_csv(std::string workspace){ //this function returns the right index values for the pos vector, for the content vektor you have to add 1 to get the correspondent line in the csv
            for(int i = 1; i<content.size(); i++){
                if(content[i][0].compare(workspace) == 0){
                    return i-1;
                }
            }
        ROS_INFO("No Line Found in CSV!");
	    return(0);
        }

};
int main(int argc, char **argv){
    ros::init(argc, argv,  "Swot_Navigation_Node");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    Navigation newgoal;
    ros::waitForShutdown();
}
