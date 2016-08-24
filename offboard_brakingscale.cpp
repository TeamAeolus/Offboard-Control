/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <fstream>
#include <iostream>
#include <boost/lexical_cast.hpp>

using namespace std;

int flag_z = 0;
int flag_land = 0;
int land_count = 0;
mavros_msgs::OpticalFlowRad current_ofrad;
void opt_cb(const mavros_msgs::OpticalFlowRad::ConstPtr& msg)
{
    current_ofrad = *msg;
	//cout << current_ofrad.distance;
    if( current_ofrad.distance >= 1.5 )
    {
        std::cout << "Z REACHED\n";
		
        flag_z = 1;
    }

	if( current_ofrad.distance < 0.35 && flag_z == 1 && land_count < 15)
	{
		land_count++;
		//cout << land_count;
	}

	if(land_count >= 15)
	{
		land_count = 0;
		std::cout << "LANDED\n";
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

	ifstream is("input.dat", ifstream::binary);

	is.seekg (0, is.beg);
	char readBuffer[2];
	float breakingScale = 0.0;

	if(argc > 1)
		breakingScale = boost::lexical_cast<float>(argv[1]);
	else
		breakingScale = 2.0;

	//float breakingScale = 2;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 100, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 100);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	ros::Subscriber opt_sub = nh.subscribe<mavros_msgs::OpticalFlowRad>
			("/px4flow/px4flow/raw/optical_flow_rad", 100, opt_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);
	
	float xThrottle = 0.0;
	float yThrottle = 0.0;	
	float zThrottle = 0.0;

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
	
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

    mavros_msgs::CommandBool arm_cmd;
	
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){

        local_pos_pub.publish(pose);
		//cout << "O\n";
		if( flag_z == 1 )
		{
			is.seekg(0, is.beg);
		    is.read((char *)&readBuffer, 3);
			
			cout << readBuffer[0] << readBuffer[1] << readBuffer[2] << "\n";
			if(readBuffer[0] == '1')
			{
				xThrottle = -0.3;
			}
			else if(readBuffer[0] == '2')
			{
				xThrottle = 0.3;
			}
			else if(readBuffer[0] == '3')
			{
				xThrottle = -(breakingScale * xThrottle);
			}
			else
			{
				xThrottle = 0.0;
			}
			

			if(readBuffer[1] == '1')
			{
				yThrottle = -0.3;
			}
			else if(readBuffer[1] == '2')
			{
				yThrottle = 0.3;
			}
			else if(readBuffer[1] == '3')
			{
				yThrottle = -(breakingScale * yThrottle);
			}
			else
			{
				yThrottle = 0.0;
			}


			if(readBuffer[2] == '1')
			{
				zThrottle = 0.5;
			}
			else if(readBuffer[2] == '2')
			{
				zThrottle = 1.5;
			}
			else
			{
				zThrottle = 0.0;
			}

			if(readBuffer[0] == '9')
			{
				xThrottle = 0.0;
				yThrottle = 0.0;
				zThrottle = 0.0;
			}

			//cout << "x: " << xThrottle << "\n";
			//cout << "y: " << yThrottle << "\n";
			
			pose.pose.position.x = xThrottle;
    		pose.pose.position.y = yThrottle;
		    pose.pose.position.z = zThrottle;

		}
		
		if(flag_land == 1)
		{
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

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
