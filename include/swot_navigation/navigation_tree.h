/**
*       navigation_tree.h
*
*       @date 01.01.2023
*       @author Joel Santos
*/

#pragma once

#include "swot_navigation/swot_robocup_navigation.h"

/**
 *      @class Nav_one
 *      @brief Action node for the first navigation part.
 *      @details This action node is responsible for initial checkings before starting the navigation.
 */

class Nav_one : public BT::SyncActionNode
{
    private:
        Navigation& navigation_;
        bool found;

    public:
        Nav_one(const std::string& name, Navigation& navigation);
        ~Nav_one() override;      
        BT::NodeStatus tick() override;            
}; 

/**
 *      @class Nav_two
 *      @brief Action node for the second navigation part.
 *      @details This action node is responsible for initial checkings before starting the navigation.
 */

class Nav_two : public BT::SyncActionNode
{
    private:
        Navigation& navigation_;

    public:
        Nav_two(const std::string& name, Navigation& navigation);
        ~Nav_two() override;      
        BT::NodeStatus tick() override;            
}; 

/**
 *      @class Nav_three
 *      @brief Action node for the third navigation part.
 *      @details This action node is responsible for initial checkings before starting the navigation.
 */

class Nav_three : public BT::SyncActionNode
{
    private:
        Navigation& navigation_;

    public:
        Nav_three(const std::string& name, Navigation& navigation);
        ~Nav_three() override;      
        BT::NodeStatus tick() override;            
}; 

/**
 *      @class Nav_four
 *      @brief Action node for fourth navigation part.
 *      @details This action node is responsible for initial checkings before starting the navigation.
 */

class Nav_four : public BT::SyncActionNode
{
    private:
        Navigation& navigation_;

    public:
        Nav_four(const std::string& name, Navigation& navigation);
        ~Nav_four() override;      
        BT::NodeStatus tick() override;            
}; 