#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <mavros_msgs/OpticalFlowRad.h>
#include "std_msgs/Float64.h"

int posX = 1;
int posY = 0.5;
int posZ = 1.5;

int flag_z = 0;
int flag_x = 0;
int flag_y = 0;

float x_acc_pos = 0.0;
float y_acc_pos = 0.0;
int count = 1;

geometry_msgs::PoseStamped current_localpos;
void locpos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_localpos = *msg;
    if( current_localpos.pose.position.x < 0.01)
	{
		x_acc_pos = 0.0;
	}

	if( current_localpos.pose.position.y < 0.01)
	{
		y_acc_pos = 0.0;
	}

    x_acc_pos = current_localpos.pose.position.x;
    y_acc_pos = current_localpos.pose.position.y;

    std::cout << "\nx: " << x_acc_pos;
    std::cout << "\ny: " << y_acc_pos;

    if(x_acc_pos >= posX)
    {
        //std::cout << "\nX Reached";
        flag_x = 1;
    }

    if(y_acc_pos >= posY)
    {
        //std::cout << "\nY Reached";
        flag_y = 1;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
 
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, locpos_cb);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    ros::Time last_request = ros::Time::now();

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok())
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
        count++;
    }

    return 0;
}
