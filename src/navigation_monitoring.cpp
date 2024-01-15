#include "swot_navigation/navigation_monitoring.h"
#include <chrono>
#include <thread>
#include <iostream>
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "actionlib_msgs/GoalStatus.h"
#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/Pose.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "ros/init.h"
#include "ros/rate.h"

#ifndef DEBUG
#define DEBUG
#endif

const float TIMEOUT = 2.0;
const int WAIT = 200;
const int RATE = 2;
const double THRESHOLD = 30.0;  // found empirically to be around 30
const bool debug = true;
const std::string NODE_NAME = "swot_navigation_monitoring";

NavigationMonitor::NavigationMonitor(ros::NodeHandle& nh) : nh_(nh), ac_("move_base", true), status_(false)
{
  teb_subscriber_ =
      nh_.subscribe("/move_base/TebLocalPlannerROS/teb_poses", 1, &NavigationMonitor::tebMonitorCallback, this);
  // Check if Action client works correctly
  while (!ac_.waitForServer(ros::Duration(TIMEOUT)))
  {
    ROS_INFO_COND(debug, "Waiting for the move_base action server to come up");
  }
}
NavigationMonitor::~NavigationMonitor() = default;


void NavigationMonitor::start(int rate)
{
  ros::Rate loop_rate(rate);
  while (ros::ok())
  {
    // evaluate current trajectories_ send new goal/cancel to movebase ac if necessary
    actionlib::SimpleClientGoalState goal_state = ac_.getState();
    std::cout << "Goal State: " << goal_state.toString() << std::endl;
    if (!evaluateTrajectories())
    {
      if (ac_.getState() != actionlib::SimpleClientGoalState::LOST)
      {
        ROS_INFO_COND(debug, "Cancelling invalid robot goal");
        ac_.cancelGoal();
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT));
      }
      std::cout << "Sending new goal: " << std::endl;
      ac_.sendGoal(createGoalMsg(0.6, 1.9, 0));
    }
    else
    {
      ROS_INFO_COND(debug, "Valid goal");
    }
    trajectories_.clear();
    trajectories_.shrink_to_fit();
    //  apperently clear does not deallocate mem, shrink to fit might if objects have been "deindexed" from the vector,
    //  but it is not guaranteed. using dynamic mem with explicit deallocation might be better
    //
    //  ros::spin();  evenutally move to getting callback once to be more efficient? however this would reduce the
    //                amount of received values to compute evaluation over
    loop_rate.sleep();
  }
}

/**
 * @brief Helper Function to create a new Goal
 * 
 * @param position_x x coordinate 
 * @param position_y y coordinate
 * @param orientation_z orientation around z axis as an euler angle
 * @return move_base_msgs::MoveBaseGoal 
 */
move_base_msgs::MoveBaseGoal NavigationMonitor::createGoalMsg(double position_x, double position_y, double orientation_z)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.seq = 1;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.pose.position.x = position_x;
  goal.target_pose.pose.position.y = position_y;
  goal.target_pose.pose.position.z = 0;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(orientation_z);
  return goal;
}

/**
 * @brief Simple Algorithm to evalute if a trajectory is valid or not. This works fairly well for most cases, however long tail behavior has to be tested
 * 
 * @return true if trajectory is valid
 * @return false if trajectory is invalid
 */
bool NavigationMonitor::evaluateTrajectories()
{
  double score = 0;
#ifdef DEBUG
  std::chrono::system_clock::time_point start = std::chrono::high_resolution_clock::now();
  std::cout << "score init: " << score << std::endl;
#endif

  // this might cause quite high load depending on publish frequency of the planner,
  // check overall performance impact later
  for (int i = 1; i < trajectories_.size(); i++)
  {
    for (int j = 0; j < trajectories_[i].poses.size(); j++)
    {
      if (j < trajectories_[i - 1].poses.size())
      {
        score += abs(trajectories_[i].poses[j].position.x - trajectories_[i - 1].poses[j].position.x);
        score += abs(trajectories_[i].poses[j].position.y - trajectories_[i - 1].poses[j].position.y);
      }
    }
  }
#ifdef DEBUG
  std::cout << "score final: " << score << std::endl;
  std::chrono::system_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> compute_duration = end - start;
  std::cout << "evaluateTrajectory Duration: " << compute_duration.count() << std::endl;
#endif

  ROS_INFO_COND(debug, "Score: %f", score);
  if (score > THRESHOLD)
  {
    ROS_INFO_COND(debug, "Exceeded Threshold");
    return false;
  }
  return true;
}

/**
 * @brief Simple callback for the current state of the Teb planner
 * 
 * @param msg returned message
 */
void NavigationMonitor::tebMonitorCallback(const geometry_msgs::PoseArray& msg)
{
  trajectories_.push_back(msg);
}


int main(int argc, char** argv)
{
  ROS_INFO("starting Node Navigation Montitoring, with Debug: %d", debug);
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh("");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  NavigationMonitor navigation_monitor(nh);
  navigation_monitor.start(RATE);
  ros::waitForShutdown();
}