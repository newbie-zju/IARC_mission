#ifndef IARCMISSION_H
#define IARCMISSION_H
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <dji_sdk/LocalPosition.h>
#include <goal_detected/Pose3D.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <dji_sdk/dji_drone.h>
#include <iarc_mission/TG.h>
#include "unistd.h"
#include <vector>
#include <iarc_tf/Velocity.h>
#include <geometry_msgs/PointStamped.h>
enum DPstate_{FREE,CRUISE,TRACK,APPROACH};//飞行器状态：巡航、跟踪、接近
enum VelState_{NED,GROUND};
#define PI 3.1415926
using namespace std;
namespace mission{
class IARCMission
{
public:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_param;
	ros::Subscriber irobot_pos_sub;	//target position from computer vision (package = goal_detected)
	ros::Subscriber dji_local_pos_sub;	//local position of DJI in NED fram (package = dji_sdk)
	ros::Subscriber flight_ctrl_dst_sub;
	ros::Subscriber obstacleAvoidance_sub;
	ros::Subscriber boundaryDetect_sub;
	ros::Subscriber quadrotorPosGround_sub;
	ros::ServiceClient TG_client;
	ros::ServiceClient tf_vel_client;
	struct irobotPosNED_
	{
		double x;
		double y;
		double z;
		double theta;
		int8_t flag;
	};
	struct irobotsPosNEDWithReward_
	{
		vector<double> x;
		vector<double> y;
		vector<double> z;
		vector<double> theta;
		int8_t flag;
		vector<double> reward;
	};
	irobotPosNED_ irobotPosNED;
	irobotsPosNEDWithReward_ irobotsPosNEDWithReward;
	dji_sdk::LocalPosition localPosNED;
	geometry_msgs::Point32 flight_ctrl_dst;
	geometry_msgs::Point quadrotorGroundPos;
	std_msgs::Int8 mission_state_msg;
	ros::Time free_time;
	ros::Time free_time_prev;
	ros::Duration freeTimer;
	
	DJIDrone *CDJIDrone;
	
	bool obstacleEmergency;
	bool boundaryEmergency;	
	float yaw_origin;
	int quadState;
	struct irobotPose
	{
		float x;
		float y;
		float theta;
	};	
	IARCMission(ros::NodeHandle nh);
	~IARCMission();
	void initialize();
	void irobot_pos_callback(const goal_detected::Pose3DConstPtr &msg);
	void dji_local_pos_callback(const dji_sdk::LocalPositionConstPtr &msg);
	void obstacleAvoidance_callback(const std_msgs::Bool msg);
	void boundaryDetect_callback(const geometry_msgs::PointConstPtr &msg);
	void quadrotorPosGroundCallback(const geometry_msgs::PointStampedConstPtr& msg);
	bool mission_takeoff();
	bool mission_land();
	int stateMachine();
	bool irobotSafe(double theta);
	void missionFree();
	void missionCruise();
	void missionTrack();
	void missionApproach();
	bool gotoFree();
	bool gotoCruise();
	bool gotoTrack();
	bool gotoApproach();
	float getLength2f(float x, float y);
	irobotPose predictIrobotPose(irobotPose irobotpose, float TPred, float TInLoop);
	inline float limitAng(float theta);
	void irobotReward();
};

};
#endif
