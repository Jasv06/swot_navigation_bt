/**
*       navigation_tree.cpp
*
*       @date 01.01.2023
*       @author Joel Santos
*/

#include "swot_navigation/navigation_tree.h"


**
 *      @brief Constructor of the Nav_one class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param navigation The Navigation class object to access the neccesary data.
 */

Nav_one::Nav_one(const std::string& name, Navigation& navigation) : BT::SyncActionNode(name, {}), navigation_(navigation) 
{

}

/**
 * 	    @brief Destructor of class Nav_one.
 */

Nav_one::~Nav_one() = default;      

/**
 *      @brief Executes the tick operation of the node Nav_one.
 *      @return The execution status of the node which in this case can be SUCCESS or FAILURE.
 */

BT::NodeStatus Nav_one::tick() 
{
    std::string GOAL_WS_ID = navigation_.get_request().destination;
    bool found = std::any_of(navigation_.get_content().begin() + 1, navigation_.get_content().end(), [&GOAL_WS_ID](const auto& row) {return row[0] == GOAL_WS_ID;});
    
    if (!found) 
    {
        if (global_print_debug) 
        {
            ROS_WARN_STREAM("The Goal " << GOAL_WS_ID << " doesn't exist in the CSV_File. Service Failed");
        }
        navigation_.set_response("FAILED")
        return BT::NodeStatus::FAILURE;
    }

    if (global_print_debug) 
    {
        ROS_INFO_STREAM("Got new Goal! The next destination is: " << GOAL_WS_ID);
    }

    if (navigation_.get_near_by_WS()) 
    {
        if (global_print_debug) 
        {
            ROS_INFO("Doing a Pushback to the previous NAV GOAL of the Robot in order to Navigate Normally!");
        }
        navigation_.controll_Posecontroller(navigation_.get_Current_WS(), true);
        if (global_print_debug)
        {
            ROS_INFO("Pushback Completed");
        }
        navigation_.set_near_by_WS(false);
    }
    return BT::NodeStatus::SUCCESS; 
}  

**
 *      @brief Constructor of the Nav_two class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param navigation The Navigation class object to access the neccesary data.
 */

Nav_two::Nav_two(const std::string& name, Navigation& navigation) : BT::SyncActionNode(name, {}), navigation_(navigation) 
{

}

/**
 * 	    @brief Destructor of class Nav_two.
 */

Nav_two::~Nav_two() = default;      

/**
 *      @brief Executes the tick operation of the node Nav_two.
 *      @return The execution status of the node which in this case can be SUCCESS or FAILURE.
 */

BT::NodeStatus Nav_two::tick() 
{
   if (navState == NavigationState::DRIVE_TO_FINISHED) {
        if (GOAL_WS_ID.compare("FINISHED") == 0) {
            if (global_print_debug) {
                ROS_INFO("Robot is driving to the Endpose!");
            }

            send_nav_goal(search_line_in_csv("FINISHED"));

            if (!ac.waitForResult(ros::Duration(time_to_navigate))) {
                // First attempt failed due to timeout
                if (global_print_debug) {
                    ROS_WARN("Navigation Task to the Endpose failed for the first Time due to Timeout!");
                    ROS_INFO("Sending the Endpose NAV_GOAL again!");
                }
                send_nav_goal(search_line_in_csv("FINISHED"));
                if (!ac.waitForResult(ros::Duration(time_to_navigate))) {
                    // Second attempt failed as well
                    if (global_print_debug) {
                        ROS_WARN("Navigation Failed for the second Time due to Timeout!");
                        ROS_ERROR("Cancel the Navigation Task!!");
                    }
                    ac.cancelGoal();
                    res.status = "FAILED";
                    Current_WS = "nan";
                    resetNavigationState();  // Reset state for the next execution
                    return BehaviorStatus::Failure;
                } else {
                    // Second attempt succeeded or failed for some other reason
                    navState = NavigationState::HANDLE_NAV_RESULT;
                    return BehaviorStatus::Running;
                }
            } else {
                // First attempt succeeded or failed for some other reason
                navState = NavigationState::HANDLE_NAV_RESULT;
                return BehaviorStatus::Running;
            }
        } else {
            resetNavigationState();  // Reset state if the goal is not FINISHED
            return BehaviorStatus::Success;
        }
    }

    return BehaviorStatus::Success;  // Indicate that the goal was not FINISHED
}  

**
 *      @brief Constructor of the Nav_three class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param navigation The Navigation class object to access the neccesary data.
 */

Nav_three::Nav_three(const std::string& name, Navigation& navigation) : BT::SyncActionNode(name, {}), navigation_(navigation) 
{

}

/**
 * 	    @brief Destructor of class Nav_three.
 */

Nav_three::~Nav_three() = default;      

/**
 *      @brief Executes the tick operation of the node Nav_three.
 *      @return The execution status of the node which in this case can be SUCCESS or FAILURE.
 */

BT::NodeStatus Nav_three::tick() 
{
    if(GOAL_WS_ID.compare("RECOVERY")==0){
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
                        //TODO: Experimental; Call makeplan to calculate distances
                        //make_plan(true);
                        return true;
                    }
                    if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
                        if(global_print_debug == true){
                            ROS_WARN("Robot can't reach the Recovery Position!!");
                            ROS_ERROR("Cancel NAV_GOAL and Service!!");
                        }
                        ac.cancelGoal();
                        res.status = "FAILED";
                        Current_WS = "nan";
                        //TODO: Experimental; Call makeplan to calculate distances
                        //make_plan(true);
                        return true;
                    }
                }
            }
}  
