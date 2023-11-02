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
double lin_vel = 0.2;
double error;

class Logic
{
private:
	// Params
	int marker_ids[4] = {11, 12, 13, 15};
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
		mrk_sub = nh.subscribe("/rosbot/aruco_marker", 1000, &Logic::marker_callback, this);
		
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
		search_id_pub = nh.advertise<std_msgs::Int32>("/rosbot/search_id", 10);
	
	}
	void marker_callback(const lab_assignment::Marker msg)
	{
		//use the controller to correct the position of the robot
		controller(msg);
		cout << marker_ids[index] << endl;
		//search new marker
		if(msg.id == -1) {
			set_target.data = marker_ids[index];
			search_id_pub.publish(set_target); //set target tramite un pub, così ne cerca solo uno, vedi 13 e 15 insieme
			move_rosbot(0.0, ang_vel);
		}
		//if target found -> go next
		else if(msg.id == marker_ids[index] && msg.area >= target_area_size){ //vedi se c'è un modo migliore per farlo anzichè con l'area
			index++;
			move_rosbot(-0.4, 0.0); //altrimenti non riesce mai a trovare il prossimo token e gira all'infinito
			sleep(1);
			ang_vel = 0.8;
			set_target.data = marker_ids[index];
			search_id_pub.publish(set_target);
		}
		//if center -> move forward, else -> correct position	
		else if(msg.id == marker_ids[index] && msg.center.x < (320 + 10) && msg.center.x > (320 - 10))
		{
			move_rosbot(lin_vel, 0.0);
		}
		else{
			move_rosbot(0.0, ang_vel);
		}
	}
	
	//simple proportional controller to get the center of the marker
	void controller(const lab_assignment::Marker msg){
		if(msg.id == marker_ids[index]) {
			error = img_center - msg.center.x;
			ang_vel = error * gain;
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
