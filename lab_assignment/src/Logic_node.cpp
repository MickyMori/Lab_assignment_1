using namespace std;
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>
#include <lab_assignment/Marker.h>

const double img_center = 320;
const double gain = 0.01;
const double target_area_size = 200*200;
double ang_vel = 0.8;
double lin_vel = 0.5;
double error;

class Logic
{
private:
	// Params
	int marker_ids[4] = {11,12,13,15};;
	int index = 0;
	bool flag = false;
	std_msgs::Int32 set_target;
	// Create a node handle
	ros::NodeHandle nh;
	// Create pub and sub
	ros::Subscriber mrk_sub;
	ros::Publisher cmd_vel_pub;
	ros::Publisher search_id_pub;
public:
	Logic(): nh("~")
	{
		mrk_sub = nh.subscribe("/rosbot/aruco_marker", 1000, &Logic::marker_callback,this);
		
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
		search_id_pub = nh.advertise<std_msgs::Int32>("/rosbot/search_id", 10);
	
	}
	void marker_callback(const lab_assignment::Marker msg)
	{
		controller(msg);
		cout << marker_ids[index] << endl;
		if(msg.id == -1)
			set_target.data = marker_ids[index];
			search_id_pub.publish(set_target);
			move_rosbot(0.0, ang_vel);
		if(msg.area >= target_area_size){
			move_rosbot(-0.4, 0.0);
			sleep(1);
			ang_vel = 0.8;
			index++;
			set_target.data = marker_ids[index];
			search_id_pub.publish(set_target);
			if(marker_ids[index] == 15)
				flag = false;
		}	
		if(msg.center.x < (320 + 10) && msg.center.x > (320 - 10))
		{
			move_rosbot(lin_vel, 0.0);
		}else{
			move_rosbot(0.0, ang_vel);
		}
		
	}
	
	void controller(const lab_assignment::Marker msg){
		error = img_center - msg.center.x;
		ang_vel = error * gain;
	}

	void move_rosbot(double lin_x,double ang_z)
	{

		geometry_msgs::Twist cmd_vel_msg;
	    	cmd_vel_msg.angular.z = ang_z;
	    	cmd_vel_msg.linear.x = lin_x;
		cmd_vel_pub.publish(cmd_vel_msg);
		
	}
};
int main(int argc, char** argv)
{
	// Initialize the ROS node
	ros::init(argc, argv, "logic_node");
	cout << "Logic node starting";
	flush(cout);
	
	Logic node;

	ros::spin();
}
