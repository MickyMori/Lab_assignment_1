/**
 * @file Logic_node.cpp
 * @brief ROS node for controlling a robot's movement based on Aruco markers.
 */

// Including necessary C++ and ROS libraries
#include <iostream> /**< Standard input/output stream. */
#include <ros/ros.h> /**< ROS (Robot Operating System) library. */
#include <std_msgs/Int32.h> /**< ROS standard integer message type. */
#include <geometry_msgs/Twist.h> /**< ROS geometry twist message type. */
#include <unistd.h> /**< Standard symbolic constants and types. */
#include <lab_assignment/Marker.h> /**< Custom message type for Aruco markers. */

// Constants for image processing and control
const double img_center = 320; /**< Horizontal center of the image. */
const double gain = 0.01; /**< Proportional gain for the controller. */
const double target_area_size = 24000; /**< Target area size for marker detection. */
const int number_of_markers = 4; /**< Numer of markers present in the environment */
double ang_vel = 0.8; /**< Angular velocity for robot movement. */
double lin_vel = 0.2; /**< Linear velocity for robot movement. */
double error; /**< Error for the proportional controller. */

// Class definition for the logic of the robot
class Logic
{
private:
    // Params
    int marker_ids[4] = {11, 12, 13, 15}; /**< Array of marker IDs to be detected. */
    int index = 0; /**< Index to track the current marker in the array. */
    bool flag = false; /**< Flag variable. */
    std_msgs::Int32 set_target; /**< Message to set the target marker ID. */

    // Create a node handle
    ros::NodeHandle nh; /**< ROS NodeHandle for communication. */

    // Create pub and sub
    ros::Subscriber mrk_sub; /**< ROS subscriber for the Aruco marker topic. */
    ros::Publisher cmd_vel_pub; /**< ROS publisher for robot velocity commands. */
    ros::Publisher search_id_pub; /**< ROS publisher for searching a specific marker ID. */

public:
    // Constructor for the Logic class
    /**
     * @brief Constructs a Logic object and initializes ROS node, publishers, and subscribers.
     */
    Logic() : nh("~")
    {
        // Subscribe to marker topic
        mrk_sub = nh.subscribe("/rosbot/aruco_marker", 1000, &Logic::marker_callback, this);

        // Advertise publishers for robot control
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        search_id_pub = nh.advertise<std_msgs::Int32>("/rosbot/search_id", 10);
    }

    // Callback function for the marker topic
    /**
     * @brief Callback function for the Aruco marker topic. Handles marker detection and robot movement.
     * @param msg The marker message containing ID, area, and position information.
     */
    void marker_callback(const lab_assignment::Marker msg)
    {
        // Use the controller to correct the position of the robot
        controller(msg);

        // Search for a new marker
        if (msg.id == -1)
        {
            set_target.data = marker_ids[index];
            search_id_pub.publish(set_target);
            move_rosbot(0.0, ang_vel);
        }
        // If target found, go to the next one
        else if (msg.area >= target_area_size)
        {
            index++;
            //if the rosbot found all the markers shutdown this node
            if(index == number_of_markers)
            {
            	move_rosbot(0.0, 0.0);
            	ros::shutdown();
            }
            ang_vel = 0.8;
            move_rosbot(0.0, ang_vel);
            set_target.data = marker_ids[index];
            search_id_pub.publish(set_target);
        }
        // If the center is aligned, move forward; else, correct position
        else if (msg.center.x < (320 + 10) && msg.center.x > (320 - 10))
        {
            move_rosbot(lin_vel, 0.0);
        }
        else
        {
            move_rosbot(0.0, ang_vel);
        }
    }

    // Simple proportional controller to get the center of the marker
    /**
     * @brief Simple proportional controller to adjust the robot's orientation based on the marker's position.
     * @param msg The marker message containing ID, area, and position information.
     */
    void controller(const lab_assignment::Marker msg)
    {
        if (msg.id == marker_ids[index])
        {
            error = img_center - msg.center.x;
            ang_vel = error * gain;
        }
    }

    // Move the robot using the provided linear and angular velocities
    /**
     * @brief Moves the robot by publishing linear and angular velocities to the cmd_vel topic.
     * @param lin_x Linear velocity along the x-axis.
     * @param ang_z Angular velocity around the z-axis.
     */
    void move_rosbot(double lin_x, double ang_z)
    {
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.angular.z = ang_z;
        cmd_vel_msg.linear.x = lin_x;
        cmd_vel_pub.publish(cmd_vel_msg);
    }
};

// Main function
/**
 * @brief The main function initializes the ROS node and creates an instance of the Logic class.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit status.
 */
int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "logic_node");

    // Create an instance of the Logic class
    Logic node;

    // Enter the ROS spin loop
    ros::spin();
}
