#ifndef IARCMISSION_H
#define IARCMISSION_H
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <iostream>
#include <dji_sdk/LocalPosition.h>
#include <goal_detected/Pose3D.h>
#include <geometry_msgs/Point32.h>
#include <dji_sdk/dji_drone.h>
#include <iarc_mission/TG.h>
#include "unistd.h"
enum DPstate{FREE,CRUISE,TRACK,APPROACH};//飞行器状态：巡航、跟踪、接近
#define PI 3.1415926
namespace mission{
class IARCMission
{
public:
	ros::NodeHandle nh_;
	ros::Subscriber irobot_pos_sub;	//target position from computer vision (package = goal_detected)
	ros::Subscriber dji_local_pos_sub;	//local position of DJI in NED fram (package = dji_sdk)
	ros::Subscriber flight_ctrl_dst_sub;
	//ros::Publisher mission_state_pub;	//mision state of quadrotor, publish to trajectory generator (mission state: CRUISE, TRACK, APPROACH)
	ros::ServiceClient TG_client;
	goal_detected::Pose3D irobotPosNED;
	dji_sdk::LocalPosition localPosNED;
	geometry_msgs::Point32 flight_ctrl_dst;
	DPstate mission_state;
	std_msgs::Int8 mission_state_msg;
	DJIDrone *CDJIDrone;
	
	IARCMission(ros::NodeHandle nh);
	~IARCMission();
	void irobot_pos_callback(const goal_detected::Pose3DConstPtr &msg);
	void dji_local_pos_callback(const dji_sdk::LocalPositionConstPtr &msg);
	void flight_ctrl_dst_callback(const geometry_msgs::Point32ConstPtr &msg);
};

};
#endif