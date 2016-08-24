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

int flag_z = 0;
int flag_land = 0;
int land_count = 0;

mavros_msgs::OpticalFlowRad current_ofrad;
void opt_cb(const mavros_msgs::OpticalFlowRad::ConstPtr& msg)
{
    current_ofrad = *msg;

    if( current_ofrad.distance > 1 )
    {
        std::cout << "Z REACHED\n";
        flag_z = 1;
    }

	if( current_ofrad.distance <= 0.33 && flag_z == 1 && land_count < 100)
	{
		land_count++;
	}

	if( land_count >= 100 )
	{
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

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 100, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 100);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	ros::Subscriber opt_sub = nh.subscribe<mavros_msgs::OpticalFlowRad>("/px4flow/px4flow/raw/optical_flow_rad", 100, opt_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);

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

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

		if( flag_z == 1 )
		{
		    pose.pose.position.x = 0;
		    pose.pose.position.y = 0;
		    pose.pose.position.z = 0;
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
