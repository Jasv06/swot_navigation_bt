/**
*       swot_robocup_navigation.cpp
*
*       @date 01.01.2023
*       @author Joel Santos
*/

#include "swot_navigation/swot_robocup_navigation.h"


Navigation::Navigation() : ac("move_base", true)
{
    executing = false;
    near_by_WS = false;
    Current_WS = "FINISHED";
    CSV_RECOVERY = false;
    CSV_FINISHED = false;
    ID_CSV = 0;
    time_to_navigate = 45;
    navState = NavigationState::DRIVE_TO_FINISHED;

    // Check if Action client works correctly
    waitForMoveBase();

    // Define service_server
    service_server = nh.advertiseService("swot_next_destination", &Navigation::callback_service_navigation, this);

    // Define PoseController client
    Pose_Controller_Client = nh.serviceClient<swot_msgs::SwotPoseController>("PoseControllerServer");

    // Read CSV File
    readCSVFile();

    // Process CSV data
    processCSVData();

    // Check CSV flags
    checkCSVFlags();

    // Initialize plan
    makePlan(false, "FINISHED"); // TODO: Experimental, to have the distances already from the start position

    // Print status
    printStatus();

}

void Navigation::waitForMoveBase() 
{
    while (!ac.waitForServer(ros::Duration(5.0))) 
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
}

void Navigation::readCSVFile() 
{
    csv_file_path = ros::package::getPath("swot_navigation") + "/src/WS_Positions.csv";
    if (global_print_debug)
    {
        ROS_INFO_STREAM("Filepath: " << csv_file_path);
    }

    std::vector<std::string> row;
    std::string line, word;
    std::fstream file(csv_file_path, std::ios::in);

    if (!file.is_open()) 
    {
        if (global_print_debug) 
        {
            ROS_ERROR("Could not open CSV File!!!");
        }
        ros::shutdown();
    }

    while (getline(file, line)) 
    {
        row.clear();
        std::stringstream str(line);
        while (getline(str, word, ',')) 
        {
            row.push_back(word);
        }
        content.push_back(row);
    }
}

void Navigation::processCSVData() 
{
    for (size_t i = 1; i < content.size(); i++) 
    {
        std::vector<float> dummy;
        for (size_t j = 1; j < content[i].size(); j++) 
        {
            dummy.push_back(std::stof(content[i][j]));
        }
        pos.push_back(dummy);
    }
}

void Navigation::checkCSVFlags() 
{
    for (size_t i = 1; i < content.size(); i++) 
    {
        if (content[i][0] == "RECOVERY") 
        {
            CSV_RECOVERY = true;
        }
        if (content[i][0] == "FINISHED") 
        {
            CSV_FINISHED = true;
        }
    }

    if (!CSV_RECOVERY || !CSV_FINISHED) 
    {
        if (global_print_debug) 
        {
            ROS_ERROR("Fehlerhafte CSV");
        }
        ros::shutdown();
    }
}

void Navigation::printStatus() 
{
    if (global_print_debug) 
    {
        ROS_INFO("Navigation ready for Competition!!!");
    }
}


//This is the function that gets called everytime the service is called
bool Navigation::callback_service_navigation(swot_msgs::SwotNavigation::Request  &ws_message, swot_msgs::SwotNavigation::Response &res)
{
    this->request_ = ws_message;
    this->response_ = res;

    BT::BehaviorTreeFactory factory;
    registerNodes(factory, *this);   
    std::string package_path = ros::package::getPath("swot_navigation");
    std::string xml_file_path = package_path + "/xml_structure/swot_navigation.xml";
    nh_.param<std::string>("file", xml_file, xml_file_path);
    auto tree = factory.createTreeFromFile(xml_file);
    tree.tickOnce();
    std::cout << get_tree_status() << std::endl;
    return true;
}

void Navigation::registerNodes(BT::BehaviorTreeFactory& factory, Navigation& navigation)
{
    BT::NodeBuilder builder_1 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<CheckObjRequired>(name,  std::ref(navigation));};
    factory.registerBuilder<CheckObjRequired>("CheckObjRequired", builder_1);

    BT::NodeBuilder builder_2 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<CheckWSFree>(name, std::ref(navigation));};
    factory.registerBuilder<CheckWSFree>("CheckWSFree", builder_2);

    BT::NodeBuilder builder_3 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<DropObjectInTray>(name,  std::ref(navigation));};
    factory.registerBuilder<DropObjectInTray>("DropObjectInTray", builder_3);

    BT::NodeBuilder builder_4 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<GetGraspAndMoveGrasp>(name,  std::ref(navigation));};
    factory.registerBuilder<GetGraspAndMoveGrasp>("GetGraspAndMoveGrasp", builder_4);

    BT::NodeBuilder builder_5 = [&](const std::string& name, const BT::NodeConfiguration& config) {return std::make_unique<MoveHomePos>(name,  std::ref(navigation));};
    factory.registerBuilder<MoveHomePos>("MoveHomePos", builder_5);
}

//This function sends navigation goals
void Navigation::send_nav_goal(int CSV_Line)
{
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

//This function handles the driving to multiple Controllerpositions and the Pushbacks
void Navigation::control_Posecontroller(const std::string& workspace, bool pushback) {
    if (workspace == "RECOVERY" || workspace == "FINISHED") {
        if (global_print_debug) {
            ROS_INFO_STREAM("No Controller Goal for Driving to " << workspace << "!");
        }
        return;
    }

    int number_of_controller_positions = std::count_if(content.begin() + 1, content.end(), 
        &workspace { return row[0].find(workspace + "_controller") != std::string::npos; });

    if (global_print_debug) {
        ROS_INFO_STREAM("Number of controller Positions: " << number_of_controller_positions);
    }

    if (!pushback || number_of_controller_positions == 1) {
        send_controller_goal(search_line_in_csv(workspace));
    } else {
        for (int j = number_of_controller_positions; j > 0; --j) {
            std::string string_to_send = workspace + "_controller" + std::to_string(j);
            send_controller_goal(search_line_in_csv(string_to_send));
        }
    }
}

//This function sends controller goals
void Navigation::send_controller_goal(int CSV_Line)
{
     if(global_print_debug == true)
     {
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
    if(Pose_Controller_Client.call(srv))
    {
        if(!srv.response.status.compare("FINISHED"))
        {
            if(global_print_debug == true)
            {
                ROS_INFO(" ");
            }
        }
    }
}

//This function returns the right index values for the pos vector, for the content vektor you have to add 1 to get the correspondent line in the csv
int Navigation::search_line_in_csv(std::string workspace)
{
    for(int i = 1; i<content.size(); i++)
    {
        if(content[i][0].compare(workspace) == 0)
        {
                    return i-1;
        }
    }
    ROS_INFO("No Line Found in CSV!");
    return(0);
}


//This function makes a plan
void Navigation::makePlan(bool setZero=false, std::string ws_goal="FINISHED")
{
    return;
}

//set functions
void Navigation::set_response(std::string response)
{
    this->response_.destination = response;
}

void Navigation::set_executing(bool exec)
{
    this->executing = exec;
}

void Navigation::set_near_by_WS(bool near)
{
    this->near_by_WS = near;
}

void Navigation::set_Current_WS(std::string current)
{
    this->Current_WS = current;
}

void Navigation::set_CSV_RECOVERY(bool csv)
{
    this->CSV_RECOVERY = csv;
}

void Navigation::set_CSV_FINISHED(bool csv)
{
    this->CSV_FINISHED = csv;
}

 void Navigation::set_tree_status(bool tree)
 {
    this->tree_status = tree;
 }

//get functions
swot_msgs::SwotNavigation::Request Navigation::get_request()
{
    return this->request_;
}

Client Navigation::get_ac()
{
    return ac;
}

bool Navigation::get_executing()
{
    return this->executing;
}

bool Navigation::get_near_by_WS()
{
    return this->near_by_WS;
}

std::string Navigation::get_Current_WS()
{
    return this->Current_WS;
}

bool Navigation::get_CSV_RECOVERY()
{
    return this->CSV_RECOVERY;
}

bool Navigation::get_CSV_FINISHED()
{
    return this->CSV_FINISHED;
}

std::vector<std::vector<std::string>> Navigation::get_content()
{
    return this->content;
}

bool Navigation::get_tree_status()
{
    return this->tree_status;
}

float Navigation::get_time_to_navigate()
{
    return this->time_to_navigate;
}

NavigationState Navigation::get_navState()
{
    return this->navState;
}
        
// Reset the navigation state
void Navigation::resetNavigationState() 
{
    navState = NavigationState::DRIVE_TO_FINISHED;
}

// Tick function for handling the navigation result
BT::NodeStatus Navigation::tickHandleNavigationResult()
{
    if (navState == NavigationState::HANDLE_NAV_RESULT) 
    {
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
        {
            res.status = "FINISHED";
            near_by_WS = false;
            Current_WS = "FINISHED";
            resetNavigationState();  // Reset state for the next execution
            return BehaviorStatus::Success;
        } 
        else if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED) 
        {
            send_nav_goal(search_line_in_csv("RECOVERY"));
            if (!ac.waitForResult(ros::Duration(time_to_navigate))) 
            {
                ac.cancelGoal();
                res.status = "FAILED";
                resetNavigationState();  // Reset state for the next execution
                return BehaviorStatus::Failure;
            } 
            else 
            {
                // Robot reached the Recovery Pose successfully
                if (global_print_debug) 
                {
                    ROS_INFO("Robot reached the Recovery Pose!");
                    ROS_INFO("Sending the NAV GOAL to the Finished Pose again!");
                }
                send_nav_goal(search_line_in_csv("FINISHED"));
                if (!ac.waitForResult(ros::Duration(time_to_navigate))) 
                {
                    ac.cancelGoal();
                    res.status = "FAILED";
                    resetNavigationState();  // Reset state for the next execution
                    return BehaviorStatus::Failure;
                } 
                else 
                {
                    // Navigation to FINISHED Pose from Recovery Pose succeeded or failed for some other reason
                    navState = NavigationState::HANDLE_NAV_RESULT_FROM_RECOVERY;
                    return BehaviorStatus::Running;
                }
            }
        }
    }

    return BehaviorStatus::Success;
}

// Tick function for handling the navigation result from Recovery Pose
BT::NodeStatus Navigation::tickHandleNavigationResultFromRecovery()
{
    if (navState == NavigationState::HANDLE_NAV_RESULT_FROM_RECOVERY) 
    {
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
        {
            res.status = "FINISHED";
            near_by_WS = false;
            Current_WS = "FINISHED";
            resetNavigationState();  // Reset state for the next execution
            return BehaviorStatus::Success;
        }
        else if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED) 
        {
            ac.cancelGoal();
            res.status = "FAILED";
            resetNavigationState();  // Reset state for the next execution
            return BehaviorStatus::Failure;
        }
    }
    return BehaviorStatus::Success;
}