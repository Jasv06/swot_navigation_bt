//Read target pose from specific topic and send a twist message to move to that point	
	//Moves in both x and y direction using an mnidirectional robot

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "math.h"

class PoseController {
	
	private:

		double x_position = 0;
		double y_position = 0;
		double z_rotation = 0;
		int targetReached = 2; //state machine 0: turn to goal orientation; 1: move in x and y until target point reached; 2: Goal Reached
		bool newTarget = false;
		double goal_x = 0;
		double goal_y = 0;
		double goal_rot = 0;
		bool newPose = false;
		double acceptableError_position = 0.01; //in meters
		double acceptableError_orientation = 0.01; //in rad

		ros::NodeHandle n;
		
		//Create a publisher and name the topic
		ros::Publisher robot_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		
		//Create subscriber
		//subscribe to pose topic in radian
        ros::Subscriber pose_sub = n.subscribe("/swot_euler_pose", 10, &PoseController::poseCallback, this);
        ros::Subscriber goal_sub = n.subscribe("/goal_pose_mobile", 10, &PoseController::goalCallback, this);
		
		//Tell ROS how fast to run this Node
		ros::Rate loop_rate = 10; //equals the rate of the pose publisher
		geometry_msgs::Twist vel_msg;
		
		geometry_msgs::Twist calcVector() {		
			geometry_msgs::Twist vector;
			double Kh = 0.45; //rotation
			double Kv = 0.5; //translation
			double x_speed = 0;
			double y_speed = 0;
			double rot_speed = 0;
			
			//Apply omnidirectional pose control
			if(targetReached == 0) { //Turn to target orientation
				double angle_to_rotate = goal_rot - z_rotation;
				//Robot should always take the shortest rotation direction and rotate over te +-pi barrier
				if(angle_to_rotate > M_PI) {
					angle_to_rotate = -2*M_PI+angle_to_rotate;
				} else if(angle_to_rotate < -M_PI) {
					angle_to_rotate = 2*M_PI+angle_to_rotate;
				}
				rot_speed = Kh * angle_to_rotate;
				ROS_INFO("Rotational Speed: %0.6f", rot_speed);
				ROS_INFO("z_rotation: %0.6f", z_rotation);
				ROS_INFO("goal rotation: %0.6f", goal_rot);
			} else if(targetReached == 1) { //Move in x and y direction to the target point
				//x_speed = Kv * (goal_x - x_position);
				//y_speed = Kv * (goal_y - y_position);
				//Compute distance between current Position and goal Position
				double total_distance = sqrt(pow((goal_y - y_position), 2) + pow((goal_x - x_position), 2));
				double theta_ij_rad = atan2((goal_y - y_position), (goal_x-x_position));
				ROS_INFO("theta_ij_rad: %0.6f", theta_ij_rad);
				//get distance in x and y direction
				double x_part = 0;
				double y_part = 0;
				//if(theta_ij_rad > 0 && theta_ij_rad < M_PI/2) {
				x_part = cos(theta_ij_rad-z_rotation) * total_distance;
				y_part = sin(theta_ij_rad-z_rotation) * total_distance;
				//}else if (theta_ij_rad>M_PI/2) {
				//	x_part = cos(theta_ij_rad) * total_distance;
				//	y_part = sin(theta_ij_rad) * total_distance;
				//} else if(theta_ij_rad<0 && theta_ij_rad > -M_PI/2) {
				//	x_part = cos(theta_ij_rad) * total_distance;
				//	y_part = sin(theta_ij_rad) * total_distance;
				//} else if(theta_ij_rad < -M_PI/2) {
				//	x_part = cos(theta_ij_rad) * total_distance;
				//	y_part = sin(theta_ij_rad) * total_distance;
				//}
				ROS_INFO("x_part: %0.6f", x_part);
				ROS_INFO("y_part: %0.6f", y_part);
				x_speed = Kv * x_part;
				y_speed = Kv * y_part;
			}
			
			//limit rotational speed
			if(rot_speed > 0.8) {
				rot_speed = 0.8;
			} else if (rot_speed < -0.8) {
				rot_speed = -0.8;
			}
			//Limit linear speed
			if(x_speed > 0.08) {
				x_speed = 0.08;
			} else if (x_speed < -0.08) {
				x_speed = -0.08;
			}
			if(y_speed > 0.08) {
				y_speed = 0.08;
			} else if(y_speed < -0.08) {
				y_speed = -0.08;
			}

			//ROS_INFO("----------------------------------------");
			//ROS_INFO("Angles: theta_ij:%0.6f, theta_i:%0.6f, omega_ij:%0.6f", theta_ij_deg, theta_i_deg, omega_ij);
			//ROS_INFO("Linear Speed v_ij: %0.6f", v_ij);
			//ROS_INFO("------------------------------------");
			
			vector.linear.x = x_speed;
			vector.linear.y = y_speed;
			vector.angular.z = rot_speed;
			return vector;
		}
	
	public:
	
		// Callback function for subscriber. 
		//Data type of /bolt/bolt_pose is geometry_msgs/Pose; consists of position float64 x, float64 y, float64 z and orientation float64 x, float64 y, float64 z, float64 w
		void poseCallback(const geometry_msgs::Pose& msg) {        
			x_position = msg.position.x;
			y_position = msg.position.y;
			z_rotation = msg.orientation.z;
			
			//Check if rotation is correct
			//check if  error margin traverses +-M_PI line
			double lower_limit_rot = goal_rot-acceptableError_orientation;
			double upper_limit_rot = goal_rot+acceptableError_orientation;
			bool upper_limit_changed = false;
			bool lower_limit_changed = false;
			if(lower_limit_rot < -M_PI) {
				lower_limit_rot = 2*M_PI + lower_limit_rot;
				lower_limit_changed = true;
			} else if(upper_limit_rot > M_PI) {
				upper_limit_rot = -2*M_PI + upper_limit_rot;
				upper_limit_changed = true;
			}
			//Only move through state machine when there is a target
			if(newTarget) {
				if(((z_rotation > lower_limit_rot) && (z_rotation < upper_limit_rot)) || (lower_limit_changed && (z_rotation>lower_limit_rot || z_rotation<upper_limit_rot)) || (upper_limit_changed && (z_rotation<upper_limit_rot || z_rotation>lower_limit_rot))) {
					targetReached = 1;
					ROS_INFO("rotation is correct");
					if((x_position > goal_x-acceptableError_position) && (x_position < goal_x+acceptableError_position) && (y_position > goal_y-acceptableError_position) && (y_position < goal_y+acceptableError_position)) {
						targetReached = 2;
						ROS_INFO("Target point Reached!");
						newTarget = false;
					} 
				} else {
					targetReached = 0;
					ROS_INFO("Turning to goal orientation");
				}
			}
			ROS_INFO("Robot pose: x:%0.6f, y:%0.6f, zrot:%0.6f", x_position, y_position, z_rotation); 
			ROS_INFO("Target: x:%0.6f, y:%0.6f", goal_x, goal_y);
			newPose = true;
		}
		
		
		// Callback function for new target position. 
		void goalCallback(const geometry_msgs::Pose& goal_msg) {
			newTarget = true;
			targetReached = 0;
			goal_x = goal_msg.position.x;
			goal_y = goal_msg.position.y;
			goal_rot = goal_msg.orientation.z;
			ROS_INFO("new target: x:%0.6f, y:%0.6f, rot:%0.6f", goal_x, goal_y, goal_rot);
		} 
		

		void moveToPoint() {
			while (ros::ok()) {
				//Only compute new linear and angular velocity when new pose was received
				if((targetReached!=2) && (newPose == true)) {
					vel_msg = calcVector();
					newPose = false;
				} else {
					ROS_INFO("newPose: %d", newPose);
					ROS_INFO("Target reached or no new pose");
					vel_msg.linear.x = 0;
					vel_msg.linear.y = 0;
					vel_msg.angular.z = 0;
				}
				//Publish linear and angular speed
				robot_vel_pub.publish(vel_msg);
				
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
};


int main(int argc, char **argv) {

	ros::init(argc, argv, "swot_move_to_point");

	PoseController controller;
	controller.moveToPoint();

}



