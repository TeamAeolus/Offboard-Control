#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <mavros_msgs/OpticalFlowRad.h>
#include "std_msgs/Float64.h"
#include <fstream>
#include <iostream>


int posX = 1;
int posY = 1;
int posZ = 1.5;

int flag_z = 0;
int flag_x = 0;
int flag_y = 0;
int flag_land = 0;

float groundDistance = 0.3;

float x_acc = 0.0;
float y_acc = 0.0;
mavros_msgs::OpticalFlowRad current_ofrad;

using namespace std;

float x_acc_pos = 0.0;
float y_acc_pos = 0.0;

geometry_msgs::PoseStamped current_localpos;
void locpos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_localpos = *msg;
    x_acc_pos = current_localpos.pose.position.x;
    y_acc_pos = current_localpos.pose.position.y;
    if(flag_z == 1)
    {
    	//std::cout << "\nx: " << x_acc_pos;
    	//std::cout << "\ny: " << y_acc_pos;
    }
    //std::cout << "\nz: " << z_acc_pos;
    if(x_acc_pos >= posX)
    {
        std::cout << "\nX Reached";
        flag_x = 1;
    }

    if(y_acc_pos >= posY)
    {
        std::cout << "\nY Reached";
        flag_y = 1;
    }
}

int land_count = 0;
void opt_cb(const mavros_msgs::OpticalFlowRad::ConstPtr& msg)
{
    current_ofrad = *msg;
    groundDistance = current_ofrad.distance;
    if(current_ofrad.integration_time_us > 0)
    {
        //x_acc = ((current_ofrad.integrated_y) * current_ofrad.distance * 1000000)/current_ofrad.integration_time_us;
        //y_acc = ((current_ofrad.integrated_x) * current_ofrad.distance  * 1000000)/current_ofrad.integration_time_us;
        //x_acc + = ((current_ofrad.integrated_y) * current_ofrad.distance);
        //y_acc + = ((current_ofrad.integrated_x) * current_ofrad.distance);
    }
    
    //std::cout << "\nX: " << x_acc;
    //std::cout << "\nY: " << y_acc;
    //std::cout << "\nZ: " << groundDistance;

    if( current_ofrad.distance > posZ )
    {
        std::cout << "Z REACHED\n";
        flag_z = 1;
    }

	if( current_ofrad.distance < 0.35 && flag_z == 1 && land_count < 10)
	{
		//std::cout << "LANDED\n";
		//flag_land = 1;
		land_count++;
	}

	if(land_count >= 10)
	{
		flag_land = 1;
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

	ifstream is;
	float xPos,yPos,zPos;
	xPos = yPos = zPos = 0.0f;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 20, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 20);
    ros::Subscriber opt_sub = nh.subscribe<mavros_msgs::OpticalFlowRad>("/px4flow/px4flow/raw/optical_flow_rad", 20, opt_cb);
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 20, locpos_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

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
			is.open("input.dat", ifstream::binary);
			if(!is)
			{
				cout << "File Read Error\n";
			}
			else
			{
				is.seekg(0, is.beg);
				is.read((char *)&xPos, sizeof(float));
				is.read((char *)&yPos, sizeof(float));
				is.read((char *)&zPos, sizeof(float));
				is.close();
			}
            count++;
            pose.pose.position.x = xPos;
            pose.pose.position.y = yPos;
            pose.pose.position.z = zPos;
			cout << xPos << yPos << zPos << "\n";
			//cout << count << "\n";
        }
	    
		if(flag_land == 1)
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
