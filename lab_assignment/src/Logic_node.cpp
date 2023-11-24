using namespace std;

#define _USE_MATH_DEFINES

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>
#include <lab_assignment/Marker.h>
#include <gazebo_msgs/LinkStates.h>
#include <cmath>

//parameters
const double img_center = 320;
const double gain = 0.005;
const double gain_base_cam = 10;
const double target_area_size = 25000;
const int number_of_markers = 4;
double ang_vel = 0.8;
double cam_vel = 1.0;
double lin_vel = 0.2;
//variables
double error;
double error_base_cam = 1;
double cam_orientation[3];
double base_orientation[3];
double cam_w;
double base_w;
double cam_theta;
double base_theta;

class Logic
{
private:
	// Params
	int marker_ids[4] = {11, 12, 13, 15};
	int index = 0;
	std_msgs::Int32 set_target;
	// Create a node handle
	ros::NodeHandle nh;
	// Create pub and sub
	ros::Subscriber mrk_sub;
	ros::Publisher cmd_vel_pub;
	ros::Publisher search_id_pub;
	ros::Publisher cam_vel_pub;
	ros::Subscriber link_pose_sub;
public:
	Logic(): nh("~")
	{
		mrk_sub = nh.subscribe("/rosbot/aruco_marker", 1000, &Logic::marker_callback, this);
		link_pose_sub = nh.subscribe("/gazebo/link_states", 1000, &Logic::link_states_callback, this); 
		
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
		cam_vel_pub = nh.advertise<std_msgs::Float64>("/astra_joint_velocity_controller/command", 10);
		search_id_pub = nh.advertise<std_msgs::Int32>("/rosbot/search_id", 10);
	
	}
	void marker_callback(const lab_assignment::Marker msg)
	{
		//use the controller to correct the position of the robot
		controller(msg);
		//search new marker
		if(msg.id == -1) {
			set_target.data = marker_ids[index];
			search_id_pub.publish(set_target);
			move_camera(cam_vel);
		}
		//if target found -> go next
		else if(msg.area >= target_area_size){
			index++;
			move_rosbot(0.0,0.0);
			if(index == number_of_markers){
				ros::shutdown();
			}
			cam_vel = 1.0;
			error_base_cam = 1;
			move_camera(cam_vel);
			set_target.data = marker_ids[index];
			search_id_pub.publish(set_target);
		}
		//if marker is in the center of the image align base and camera and then move forward 	
		else if((msg.center.x < (img_center + 50) && msg.center.x > (img_center - 50)))
		{
			move_camera(0.0);
			if(abs(error_base_cam)<0.01)
			{
				move_rosbot(lin_vel, 0.0);
			}else{
				align_base_cam();
				move_camera(cam_vel);
			}
		}
		else{
			move_camera(cam_vel);
		}
	}
	
	void align_base_cam(){
		error_base_cam = cam_theta - base_theta;
		ang_vel = error_base_cam * gain_base_cam;
		
		if(ang_vel > 0.25)
			ang_vel = 0.25;
		else if(ang_vel < -0.25)
			ang_vel = -0.25;
		move_rosbot(0.0, ang_vel);
	}
	
	void link_states_callback(const gazebo_msgs::LinkStates msg){
		int i = 0;
		bool found = false;
		while(true)
		{
			if(msg.name[i] == "rosbot::camera_link")
			{
				if(cam_w < 1 + 0.001 && cam_w > 1 + 0.001)
				{
					cam_theta = 0;
					cam_orientation[0] = 1;
					cam_orientation[1] = 0;
					cam_orientation[2] = 0;
				}else{
					cam_w = msg.pose[i].orientation.w;
					cam_theta = 2*acos(cam_w);
					cam_orientation[0] = msg.pose[i].orientation.x/sin(cam_theta/2);
					cam_orientation[1] = msg.pose[i].orientation.y/sin(cam_theta/2);
					cam_orientation[2] = msg.pose[i].orientation.z/sin(cam_theta/2);
					if(cam_orientation[2] < 0)
						cam_theta = -(cam_theta-2*M_PI); 
				}
				
				if(found)
					break;
				found = true;
			}
			else if(msg.name[i] == "rosbot::base_link")
			{
				if(base_w < 1 + 0.001 && base_w > 1 + 0.001)
				{
					base_theta = 0;
					base_orientation[0] = 1;
					base_orientation[1] = 0;
					base_orientation[2] = 0;
				}else{
					base_w = msg.pose[i].orientation.w;
					base_theta = 2*acos(base_w);
					base_orientation[0] = msg.pose[i].orientation.x/sin(base_theta/2);
					base_orientation[1] = msg.pose[i].orientation.y/sin(base_theta/2);
					base_orientation[2] = msg.pose[i].orientation.z/sin(base_theta/2);
					if(base_orientation[2] < 0)
						base_theta = -(base_theta-2*M_PI); 
				}
				
				if(found)
					break;
				found = true;
			}
			i++;	
		}
	}
	
	//simple proportional controller to get the center of the marker
	void controller(const lab_assignment::Marker msg){
		if(msg.id == marker_ids[index]) {
			error = img_center - msg.center.x;
			cam_vel = error * gain;
			if(cam_vel>1.5){
				cam_vel = 1.5;
			}else if(cam_vel<-1.5)
				cam_vel = -1.5;
		}
	}

	//move the robot
	void move_rosbot(double lin_x,double ang_z)
	{
		geometry_msgs::Twist cmd_vel_msg;
	    	cmd_vel_msg.angular.z = ang_z;
	    	cmd_vel_msg.linear.x = lin_x;
		cmd_vel_pub.publish(cmd_vel_msg);	
	}
	
	//move the camera
	void move_camera(double vel)
	{
		std_msgs::Float64 cam_vel_msg;
		cam_vel_msg.data = vel;
		cam_vel_pub.publish(cam_vel_msg);
	}
};

int main(int argc, char** argv)
{
	// Initialize the ROS node
	ros::init(argc, argv, "logic_node");
	sleep(4);
	
	Logic node;

	ros::spin();
}
