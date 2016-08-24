#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <mavros_msgs/OpticalFlowRad.h>
#include "std_msgs/Float64.h"

int posX = 1;
int posY = 0.3;
int posZ = 1.5;

int flag_z = 0;
int flag_1 = 0;
int flag_2 = 0;
int flag_3 = 0;
int flag_4 = 0;

float groundDistance = 0.3;

float x_acc = 0.0;
float y_acc = 0.0;
mavros_msgs::OpticalFlowRad current_ofrad;


float x_acc_pos = 0.0;
float y_acc_pos = 0.0;

geometry_msgs::PoseStamped current_localpos;
void locpos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_localpos = *msg;
    x_acc_pos = current_localpos.pose.position.x;
    y_acc_pos = current_localpos.pose.position.y;
    std::cout << "\nx: " << x_acc_pos;
    std::cout << "\ny: " << y_acc_pos;
    //std::cout << "\nz: " << z_acc_pos;
    if(x_acc_pos >= 0 && y_acc_pos >= 1)
    {
        std::cout << "\nCorner 1 Reached";
        flag_1 = 1;
    }

    if(x_acc_pos >= 1 && y_acc_pos <= 1)
    {
        std::cout << "\nCorner 2 Reached";
        flag_2 = 1;
    }

    if(x_acc_pos <= 1 && y_acc_pos >= 0)
    {
        std::cout << "\nCorner 3 Reached";
        flag_3 = 1;
    }

    if(x_acc_pos <= 0 && y_acc_pos <= 0)
    {
        std::cout << "\nCorner 4 Reached";
        flag_4 = 1;
    }
}

void opt_cb(const mavros_msgs::OpticalFlowRad::ConstPtr& msg)
{
    current_ofrad = *msg;
    groundDistance = current_ofrad.distance;

    if( current_ofrad.distance > posZ )
    {
        std::cout << "Z REACHED\n";
        flag_z = 1;
    }
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    int count = 0;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 100);
    ros::Subscriber opt_sub = nh.subscribe<mavros_msgs::OpticalFlowRad>("/px4flow/px4flow/raw/optical_flow_rad", 100, opt_cb);
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, locpos_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);

    // wait for FCU connection
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.5;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();

        if(flag_z == 1)
        {
            pose.pose.position.x = 0;
            pose.pose.position.y = 1;
            pose.pose.position.z = 1.5;
            std::cout << "0,1\n";
	    
			if(flag_1 == 1 )
			{
				pose.pose.position.x = 1;
				pose.pose.position.y = 1;           
				pose.pose.position.z = 1.5;
				std::cout << "1,1\n";
			}

			if(flag_2 == 1 )
			{
		        flag_1 = 0;
				pose.pose.position.x = 1;
				pose.pose.position.y = 0;
				pose.pose.position.z = 1.5;
			    std::cout << "1,0\n";
			}

			if(flag_3 == 1 )
			{
		        flag_2 = 0;
				pose.pose.position.x = 0;
				pose.pose.position.y = 0;
				pose.pose.position.z = 1.5;
			    std::cout << "0,0\n";
			}

			if(flag_4 == 1)
			{
				flag_3 = 0;
				pose.pose.position.x = 0;
				pose.pose.position.y = 0;
				pose.pose.position.z = 1.5;
			    std::cout << "HOVERING\n";
		        count++;
			}

		    if(count >= 200 && count < 300)
		    {
		        flag_4 = 0;
		        pose.pose.position.x = 0;
				pose.pose.position.y = 0;
				pose.pose.position.z = 0.8;
			    std::cout << "DESCENDING\n";
		        count++;
		    }
		    
		    if(count < 500 && count >= 300)
		    {
		        pose.pose.position.x = 0;
				pose.pose.position.y = 0;
				pose.pose.position.z = -0.2;
			    std::cout << "LANDING\n";
		        count++;
		    }
		}

		if(count >= 500)
		{
			mavros_msgs::CommandBool arm_cmd;
			arm_cmd.request.value = false;
			if( current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
			{
				if( arming_client.call(arm_cmd) && arm_cmd.response.success)
				{
					std::cout << "DISARMED\n";
				}
	
				last_request = ros::Time::now();
			}
		}
    }
    return 0;
}
