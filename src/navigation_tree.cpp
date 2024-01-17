/**
*       navigation_tree.cpp
*
*       @date 01.01.2023
*       @author Joel Santos
*/

#include "swot_navigation/navigation_tree.h"


/**
 *      @brief Constructor of the Nav_one class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param navigation The Navigation class object to access the neccesary data.
 */

Nav_one::Nav_one(const std::string& name, Navigation& navigation) : BT::SyncActionNode(name, {}), navigation_(navigation), found(false)
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
     //Check if the destinated Goal is contained in the CSV File
    for(int i = 1; i < navigation_.get_content().size(); i++)
    {
        if(navigation_.get_content()[i][0].compare(GOAL_WS_ID) == 0)
        {
            found = true;
        }
    }    
    //If the destinated Goal isn't containted in the CSV File Quit the Service with Response "Failed"
    if (!found) 
    {
        navigation_.set_response("FAILED");
        return BT::NodeStatus::SUCCESS;
    }
    if (navigation_.get_near_by_WS()) 
    {
        navigation_.controll_Posecontroller(navigation_.get_Current_WS(), true);
        navigation_.set_near_by_WS(false);
    }
    return BT::NodeStatus::FAILURE;
}  

/**
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
        if (navigation_.get_request().destination.compare("FINISHED") == 0) 
        {
            navigation_.send_nav_goal(navigation_.search_line_in_csv("FINISHED"));
            if (!navigation_.ac.waitForResult(ros::Duration(navigation_.get_time_to_navigate()))) 
            {
                    navigation_.send_nav_goal(navigation_.search_line_in_csv("FINISHED"));
                    if (!navigation_.ac.waitForResult(ros::Duration(navigation_.get_time_to_navigate()))) 
                    {
                        navigation_.ac.cancelGoal();
                        navigation_.set_response("FAILED");
                        navigation_.set_Current_WS("nan");             
                        return BT::NodeStatus::SUCCESS;
                    } 
                    else 
                    {
                        BT::NodeStatus var_one = navigation_.tickHandleNavigationResult();
                        if(var_one == BT::NodeStatus::SUCCESS)
                        {
                            return BT::NodeStatus::SUCCESS;
                        }
                    }
            } 
            else 
            {
                BT::NodeStatus var_two = navigation_.tickHandleNavigationTwo();
                if(var_two == BT::NodeStatus::SUCCESS)
                {
                    return BT::NodeStatus::SUCCESS;
                }
            } 
        }
    return BT::NodeStatus::FAILURE;
}  

/**
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
    if(navigation_.get_request().destination.compare("RECOVERY")==0)
    {
        navigation_.send_nav_goal(navigation_.search_line_in_csv("RECOVERY"));
        if(!navigation_.ac.waitForResult(ros::Duration(navigation_.get_time_to_navigate())))
        {
            navigation_.ac.cancelGoal();
            navigation_.set_response("FAILED");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            if(navigation_.ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {             
                navigation_.set_response("FINISHED"); 
                navigation_.set_near_by_WS(false);                                                     
                navigation_.set_Current_WS("RECOVERY");
                return BT::NodeStatus::SUCCESS;
            }
            if(navigation_.ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
            {
                navigation_.ac.cancelGoal();
                navigation_.set_response("FAILED");
                navigation_.set_Current_WS("nan");
                return BT::NodeStatus::SUCCESS;
            }
        }
    }
    return BT::NodeStatus::FAILURE;
}  

/**
 *      @brief Constructor of the Nav_four class used to initialize the corresponding member variables.
 *      @param name The behavior tree node name.
 *      @param navigation The Navigation class object to access the neccesary data.
 */

Nav_four::Nav_four(const std::string& name, Navigation& navigation) : BT::SyncActionNode(name, {}), navigation_(navigation) 
{

}

/**
 * 	    @brief Destructor of class Nav_four.
 */

Nav_four::~Nav_four() = default;      

/**
 *      @brief Executes the tick operation of the node Nav_four.
 *      @return The execution status of the node which in this case can be SUCCESS or FAILURE.
 */

BT::NodeStatus Nav_four::tick() 
{
    navigation_.send_nav_goal(navigation_.search_line_in_csv(navigation_.get_request().destination));

    if(!navigation_.ac.waitForResult(ros::Duration(navigation_.get_time_to_navigate())))
    {
        BT::NodeStatus var_three = navigation_.tickHandleNavigationThree();
        if(var_three == BT::NodeStatus::SUCCESS)
        {
            return BT::NodeStatus::SUCCESS;
        }
    }
    else
    {
        BT::NodeStatus var_four = navigation_.tickHandleNavigationFour();
        if(var_four == BT::NodeStatus::SUCCESS)
        {
            return BT::NodeStatus::SUCCESS;
        }   
    }
    return BT::NodeStatus::SUCCESS;
}  