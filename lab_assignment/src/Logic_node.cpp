using namespace std;
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>


class Logic
{
private:
	// Params
	int marker_id = 11;
	bool flag = true;
	// Create a node handle
	ros::NodeHandle nh;
	// Create pub and sub
	ros::Subscriber mrk_id_sub;
	ros::Publisher cmd_vel_pub;
public:
	Logic(): nh("~")
	{
		mrk_id_sub = nh.subscribe("/rosbot/aruco_marker/id", 1000, &Logic::marker_id_callback,this);
		
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	}
	void marker_id_callback(const std_msgs::Int32 msg)
	{
		if(msg.data == -1)
			move_rosbot(0.0, 0.5);
		if(msg.data == marker_id)
		{
			move_rosbot(0.2, 0.0);
		}else
		{
			move_rosbot(0.0, 0.5);
		}
		
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
