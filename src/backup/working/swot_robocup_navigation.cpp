#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "swot_msgs/SwotNavigation.h"
#include "actionlib_msgs/GoalID.h"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/package.h>
#include <vector>
#include "math.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "nav_msgs/Odometry.h"


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
        //ros::Subscriber goal_sub = n.subscribe("/goal_pose_mobile", 10, &PoseController::goalCallback, this);
		
		//Tell ROS how fast to run this Node
		ros::Rate loop_rate = 15; //equals the rate of the pose publisher
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
				//ROS_INFO("theta_ij_rad: %0.6f", theta_ij_rad);
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
				//ROS_INFO("x_part: %0.6f", x_part);
				//ROS_INFO("y_part: %0.6f", y_part);
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
					//ROS_INFO("rotation is correct");
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
			//ROS_INFO("Robot pose: x:%0.6f, y:%0.6f, zrot:%0.6f", x_position, y_position, z_rotation); 
			//ROS_INFO("Target: x:%0.6f, y:%0.6f", goal_x, goal_y);
			newPose = true;
		}
		
		
		// Callback function for new target position. 
		void goalCallback(const geometry_msgs::Pose goal_msg) {
			newTarget = true;
			targetReached = 0;
			goal_x = goal_msg.position.x;
			goal_y = goal_msg.position.y;
			goal_rot = goal_msg.orientation.z;
			ROS_INFO("new target: x:%0.6f, y:%0.6f, rot:%0.6f", goal_x, goal_y, goal_rot);
		} 
		

		void moveToPoint() {
			while (newTarget == true) {     //geändert Test um zu verändern dass eine endlosschleife läuft
				//Only compute new linear and angular velocity when new pose was received
				if((targetReached!=2) && (newPose == true)) {
					vel_msg = calcVector();
					newPose = false;
				} else {
					ROS_INFO("newPose: %d", newPose);
                    ROS_INFO("target: x:%0.6f, y:%0.6f, rot:%0.6f", goal_x, goal_y, goal_rot);

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


class Navigation

{

    private:

        ros::NodeHandle nh;
        ros::ServiceServer service_server;
        //Publisher

        ros::Publisher pub_goal;
        ros::Publisher pub_cancel; 
        ros::Publisher pub_vel;

        //Subscriber
        ros::Subscriber sub_amcl;
        ros::Subscriber reachgoal;

        //Controller object
        PoseController Finepositioning_controller;

        //Variables
        std::string status;

        double x = 0;
        double y = 0;
        double z = 0;
        double w = 0;
        

        bool arrived = false;
        bool calculate_distance = false;
        bool recovery = false;
        int recovery_counter = 0;


        std::string WS_ID = "nan"; //String which contains the Workspace number from the getWS function for global access
        std::string last_WS = "nan";
        //reading from csv file
        std::ifstream Workspace_Positions;
        std::string csv_file_path;
        std::vector<std::vector<std::string>> content;
        std::vector<std::vector<float>> pos;

        int ID = 0; //indicates which line from the csv file is picked out by TEB Local Planner: //Start =1; End=2
        int ID_controller = 0; //which line from the csv file is picked out by the controller


    public:

        Navigation(){
            //construct service
            service_server = nh.advertiseService("swot_next_destination", &Navigation::getWS, this);
            
            //Construct Publisher
            //Publish new Goal
            pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
            //ros::Duration(1).sleep();
            
            //Publisher for cancelling Navigation operation
            pub_cancel = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel",1);

            //Publisher auf /cmd_vel
            pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

            //Construct Subscriber
            //Subscriber to the topic with the which indicates if the goal is reached or not
            reachgoal = nh.subscribe("/move_base/result",1,&Navigation::getres,this);
            //Subscriber to amcl Pose 
            sub_amcl = nh.subscribe("/amcl_pose",1,&Navigation::getPose,this);
            
            //open and read csv file WS_Positions
            csv_file_path = ros::package::getPath("swot_navigation")+"/src/WS_Positions_2.csv";
            std::cout << csv_file_path << std::endl;
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
                std::cout<<"Could not open the file\n";
            }  

            for (int i = 1; i<content.size(); i++){ //anzahl der zeilen
                std::vector<float> dummy;
                dummy.clear();
                for(int j = 1; j<content[i].size(); j++){
                    dummy.push_back(atof(content[i][j].c_str()));
                }
                pos.push_back(dummy);
            }
            
        }

    

        void getPose(const geometry_msgs::PoseWithCovarianceStamped& pose_data){
            x = pose_data.pose.pose.position.x;
            y = pose_data.pose.pose.position.y;
            z = pose_data.pose.pose.orientation.z;
            w = pose_data.pose.pose.orientation.w;
        }

        void getres(const move_base_msgs::MoveBaseActionResult& data){
            ROS_INFO("Get Res call");
            while(data.status.status !=3){ //add comments for status 3 and 4
                if(data.status.status ==4){
                    ROS_WARN("SWOT-BOT can't reach th desired Service Area");
                    startRecovery();
                    ROS_INFO("SWOT-BOT Start recovery done");

                    return;
                }
                return;
            }
            if(data.status.status ==3){
                if(recovery == false){
                    if(WS_ID =="START"){
                        ROS_INFO("START pose reached!");
                        status = "FINISHED";
                        arrived = true;
                    }
                    else if(WS_ID == "END"){
                        ROS_INFO("END pose reached!");
                        status = "FINISHED";
                        arrived = true;
                    }
                    else{  //if start else if end else alles andere
                        ROS_INFO("Preliminary Goal reached!");
                        ROS_INFO("Driving to closer Position");
                        geometry_msgs::Pose close_goal;
                        close_goal.position.x = pos[ID+1][0];
                        close_goal.position.y = pos[ID+1][1];
                        close_goal.position.z = 0;
                        close_goal.orientation.x = 0;
                        close_goal.orientation.y = 0;
                        close_goal.orientation.z = pos[ID+1][5];
                        close_goal.orientation.w = pos[ID+1][6];
                        geometry_msgs::Pose close_goal_euler = quaternion_to_euler(close_goal);
                        Finepositioning_controller.goalCallback(close_goal_euler);
                        Finepositioning_controller.moveToPoint();
                        last_WS = WS_ID;
                        status = "FINISHED";
                        arrived = true;
                    }

                }
                else if(recovery == true){      // else reicht if the recovery ended, send the goal a second time. 
                    ROS_INFO("IN Recovery");
                    sendgoal();
                    recovery = false;
                    ROS_INFO("Recovery done");
                }
            }
            ROS_INFO("Get Res call done");
        }

        bool getWS(swot_msgs::SwotNavigation::Request  &ws_message, swot_msgs::SwotNavigation::Response &res){   
            std::string workspace_number (ws_message.destination);
            WS_ID = workspace_number;
            if(workspace_number.compare("START")==0){
                ROS_INFO("Drive to the START Pose");
                ID = 0;
            }
            else if(workspace_number.compare("END")==0){
                ROS_INFO("Drive to the END Pose");
                ID = 1;
            }
            else { //numbered Workspace
                std::cout<<"Got new goal, the next destination is: " << workspace_number<<std::endl;

                for(int i = 1; i<content.size();i++){ //Walk through all descriptions
                
                    if(content[i][0].compare(workspace_number)==0){    //decide in which line the Workspace data is located
                        ID = i-1;
                    }
                }
                if(ID == 0 || ID == 1) {
                    ROS_INFO("Error: Workspace not found!");
                }
                //std::cout<<"ID: "<<ID<<std::endl;
                //std::cout << "X"<<pos[ID][0]<<std::endl; //pos[ROW][Column]
            }
            if(last_WS.compare("nan")!=0){
                int ID_last_WS = 0;
                for(int i = 1; i<content.size();i++){ //Walk through all descriptions
            
                    if(content[i][0].compare(last_WS)==0){    //decide in which line the Workspace data is located
                        ID_last_WS = i;
                    }
                }
                geometry_msgs::Pose last_WS_pose;
                last_WS_pose.position.x = pos[ID_last_WS][0];
                last_WS_pose.position.y = pos[ID_last_WS][1];
                last_WS_pose.position.z = 0;
                last_WS_pose.orientation.x = 0;
                last_WS_pose.orientation.y = 0;
                last_WS_pose.orientation.z = pos[ID_last_WS][5];
                last_WS_pose.orientation.w = pos[ID_last_WS][6];
                geometry_msgs::Pose last_WS_pose_euler = quaternion_to_euler(last_WS_pose);
                Finepositioning_controller.goalCallback(last_WS_pose_euler);
                Finepositioning_controller.moveToPoint();
                last_WS = "nan";
                ROS_INFO("Pushback completed");    
            }
            sendgoal();

            while(arrived == false);
            if (arrived == true){
                arrived = false;
                std::cout<<"Status "<<status<<std::endl;

                if (status.compare("FINISHED")==0){
                    ROS_INFO("Navigation Task finished!");
                    calculate_distance = true;
                    res.status = "FINISHED";
                    return true;
                }

                else if(status.compare("FAILED")==0){
                    ROS_ERROR("Navigation Task failed");
                    res.status = "FAILED";
                    return true; 
                }

                else if(status.compare("TAPE")==0){
                    calculate_distance = true; 
                    res.status = "TAPE";
                    return true; 
                }
                else {
                    return true; 
                }
            }
        }
        void sendgoal(){
            geometry_msgs::PoseStamped goal;
            goal.header.seq = 1;
            goal.header.stamp = ros::Time::now();
            goal.header.frame_id = "map";
            goal.pose.position.x = pos[ID][0];
            goal.pose.position.y = pos[ID][1];
            goal.pose.position.z = pos[ID][2];
            goal.pose.orientation.x = pos[ID][3];
            goal.pose.orientation.y = pos[ID][4];
            goal.pose.orientation.z = pos[ID][5];
            goal.pose.orientation.w = pos[ID][6];
            pub_goal.publish(goal);
            std::cout<<"x: "<<goal.pose.position.x<<" Y: "<<goal.pose.position.y<<std::endl; 
            std::cout<<"New Goal is sending to Robot"<<std::endl; 

        }
        void startRecovery(){
            double start_time_recovery = ros::Time::now().toSec();
            double duration = 20;
            //nh.getParam("/swot_robocup_navigation/duration_recovery", duration);
            while(1){
                double current_time = ros::Time::now().toSec();
                double elapsed_time = current_time - start_time_recovery;
                ROS_INFO(std::string(std::string("duration")+std::to_string(elapsed_time)).c_str());
                if (elapsed_time > duration){
                    status = "FAILED";
                    arrived = true;                                             //überdenke mit Moritz
                    ROS_WARN("Recovery takes too much time, Abort Recovery");
                    actionlib_msgs::GoalID cancel_msg;
                    cancel_msg.stamp = ros::Time::now();
                    pub_cancel.publish(cancel_msg);
                    return;
                }
                else{
                    ROS_INFO("IN Second");
                    recovery = true;
                    actionlib_msgs::GoalID cancel_msg;
                    cancel_msg.stamp = ros::Time::now();
                    pub_cancel.publish(cancel_msg);
                    ROS_INFO("Start additional Recovery Behaviour");
                    ROS_INFO("At first, perform an in place rotation to clear out the costmaps");
                    geometry_msgs::Twist vel_msg;
                    double sec = ros::Time::now().toSec();
                    double sec2 = sec + 4;
                    vel_msg.linear.x = 0;
                    vel_msg.linear.y = 0;
                    vel_msg.linear.z = 0;
                    vel_msg.angular.x = 0;
                    vel_msg.angular.y = 0;
                    vel_msg.angular.z = 0.2;
                    while(1){
                        if(sec2<sec){
                            break;
                        }
                        pub_vel.publish(vel_msg);
                        sec = ros::Time::now().toSec();
                    }
                    vel_msg.angular.z = 0;
                    pub_vel.publish(vel_msg);
                    ros::Duration(1).sleep();

                    geometry_msgs::PoseStamped goal;
                    goal.header.seq = 1;
                    goal.header.stamp = ros::Time::now();
                    goal.header.frame_id = "map";

                    if(recovery_counter < 1){
                        ROS_INFO("Drive back to the START Position");
                        goal.pose.position.x = pos[0][0];
                        goal.pose.position.y = pos[0][1];
                        goal.pose.position.z = pos[0][2];
                        goal.pose.orientation.x = pos[0][3];
                        goal.pose.orientation.y = pos[0][4];
                        goal.pose.orientation.z = pos[0][5];
                        goal.pose.orientation.w = pos[0][6];
                        recovery_counter +=1;
                    }
                    else{
                        ROS_INFO("Drive to an alternative position, because START position is not reachable at the moment");
                        goal.pose.position.x = 2.244;
                        goal.pose.position.y = -2.085;
                        goal.pose.position.z = 0.0;
                        goal.pose.orientation.x = 0.0;
                        goal.pose.orientation.y = 0.0;
                        goal.pose.orientation.z = -0.291;
                        goal.pose.orientation.w = 0.957;
                        recovery_counter = 0;
                    }
                    ros::Duration(1).sleep();
                    pub_goal.publish(goal);

                }
            }
        }
        geometry_msgs::Pose quaternion_to_euler(geometry_msgs::Pose quaternion_pose){
            tf::Quaternion quat;
            tf::quaternionMsgToTF(quaternion_pose.orientation, quat);

            // the tf::Quaternion has a method to acess roll pitch and yaw
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            // the found angles are written in a geometry_msgs::Vector3
            geometry_msgs::Pose euler_pose;
            euler_pose.position.x = quaternion_pose.position.x;
            euler_pose.position.y = quaternion_pose.position.y;
            euler_pose.position.z = 0; 
            euler_pose.orientation.x = 0;//roll;
            euler_pose.orientation.y = 0;//pitch;
            euler_pose.orientation.z = yaw;
            return euler_pose;
        } 
};


int main(int argc, char **argv){

    ros::init(argc, argv,  "Swot_Navigation_Node");
    ros::AsyncSpinner spinner(4); 
    spinner.start();
    Navigation newgoal;
    //ros::spinOnce();
    ros::waitForShutdown();
}

