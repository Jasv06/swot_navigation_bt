/****************************************************************************

Conversion from a quaternion to roll, pitch and yaw.

Nodes:
subscribed /amcl_pose oder /odom (message of type geometry_msgs::PoseWithCovarianceStamped or nav_msgs::Odometry)
published /swot_euler_pose (message oftype geometry_msgs::Pose)

****************************************************************************/

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

// Here I use global publisher and subscriber, since I want to access the
// publisher in the function MsgCallback:
ros::Publisher swot_euler_pose_publisher;
ros::Subscriber quaternion_pose_subscriber;

// Function for conversion of quaternion to roll pitch and yaw. The angles
// are published here too.
void MsgCallback(const geometry_msgs::PoseWithCovarianceStamped msg) {
//void MsgCallback(const nav_msgs::Odometry msg)
//{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Pose euler_pose;
    euler_pose.position.x = msg.pose.pose.position.x;
    euler_pose.position.y = msg.pose.pose.position.y;
    euler_pose.position.z = 0; 
    euler_pose.orientation.x = 0;//roll;
    euler_pose.orientation.y = 0;//pitch;
    euler_pose.orientation.z = yaw;

    // this Vector is then published:
    swot_euler_pose_publisher.publish(euler_pose);
    ROS_INFO("published euler angles: yaw=%f", euler_pose.orientation.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swot_euler_pose_publisher");
    ros::NodeHandle n;
    swot_euler_pose_publisher = n.advertise<geometry_msgs::Pose>("swot_euler_pose", 1);
    //quaternion_pose_subscriber = n.subscribe("/odom", 1, MsgCallback);
    quaternion_pose_subscriber = n.subscribe("/amcl_pose", 1, MsgCallback);

    // check for incoming quaternions untill ctrl+c is pressed
    ROS_INFO("waiting for quaternion");
    ros::spin();
    return 0;
}
