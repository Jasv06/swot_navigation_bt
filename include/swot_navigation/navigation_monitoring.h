
#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include <actionlib/client/simple_action_client.h>
#include "actionlib_msgs/GoalID.h"
#include "move_base_msgs/MoveBaseAction.h"

/**
 * @brief
 *
 */
/*!
 *	@class NavigationMonitor
 * 	@author Julian Müller
 *	@version 1.0
 *	@date June 20th 2023
 *	@brief PoC of a monitoring and control component for the navigation stack.
 *	@details Even though the ROS Navigation stack has a complex architecture with defined recovery behaviors,
 *           undesirable behaviors sometimes emerge that could be monitored and reacted to by an external 3rd party
 *component. The purpose of this node is to provide a proof of concept of this functionality and evaluate its usefulness
 *for improving overall navigation robustness in the RoboCup.
 */
class NavigationMonitor
{
public:
  NavigationMonitor();
  explicit NavigationMonitor(ros::NodeHandle& nh);
  ~NavigationMonitor();
  /**
   * @brief starts the monitoring loop at a desired rate, runs as long as ROS core is active.
   *
   * @param rate rate in Hz at which the control loop should run.
   * @details Currently only the TEB Planner is beeing monitored for unsual replanning behavior that sometimes occurs if
   * a feasable plan can not be found easily.
   *
   */
  void start(int rate);

private:
  bool evaluateTrajectories();
  void tebMonitorCallback(const geometry_msgs::PoseArray& msg);
  move_base_msgs::MoveBaseGoal createGoalMsg(double position_x, double position_y, double orientation_z);

  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      ac_;                          // Create a ac to interact with move base and cancel goals
  ros::Subscriber teb_subscriber_;  // Create subscriber for teb feedback containing planned path trajectories
  bool status_;
  std::vector<geometry_msgs::PoseArray> trajectories_;
};
