#include <ros/ros.h>
#include <IARCMission.h>
#include <math.h>
using namespace std;
//using namespace DJI::onboardSDK;
namespace mission{
IARCMission::IARCMission(ros::NodeHandle nh):nh_(nh),nh_param("~")
{
	double yaw_origin_;
	if(!nh_param.getParam("yaw_origin",yaw_origin_))yaw_origin_ = 0.0;
	yaw_origin = (float)yaw_origin_*180.0/M_PI;
	irobot_pos_sub = nh_.subscribe("/goal_detected/goal_pose", 10, &IARCMission::irobot_pos_callback, this);
	dji_local_pos_sub = nh_.subscribe("/dji_sdk/local_position", 10, &IARCMission::dji_local_pos_callback, this);
	TG_client = nh_.serviceClient<iarc_mission::TG>("/TG/TG_service");
	CDJIDrone = new DJIDrone(nh_);
	while(ros::ok())
	{
		ros::spinOnce();
		char operate_mode = getchar();	//waiting for start = space
		switch(operate_mode)
		{
			case 27:	//enter space to start mission
			{
				CDJIDrone->request_sdk_permission_control();
				mission_takeoff();
// 				sleep(3);	//waiting for quadrotor to arrive 1.2m
// 				for(int i = 0; i != 500;i++)
// 				{
// 					CDJIDrone->local_position_control(localPosNED.x, localPosNED.y, 1.8, 0);	//TODO: YAW!!!
// 					usleep(20000);
// 				}
				ROS_INFO("takeoff...");
				ros::Rate loop_rate(50);
				while(ros::ok())
				{
					ros::spinOnce();
					if(irobotPosNED.flag > 0)	//has irobot
					{
						ROS_INFO("has irobot");
						// TODO:define the competition area:
						//
						//			white						^  N theta = 0
						//red					green			|
						//red					green		------	|------> E theta = PI/2
						//			white						| S theta = PI
						
						//TODO: define the range of theta which means RED, GREEN, WHITE borders
						if((-0.25 < irobotPosNED.theta) && (irobotPosNED.theta < 0.25))	//PI/4 < theta <3PI/4 means irobot is going forward to green borders, than TRACK!	
						{
							iarc_mission::TG TG_srv;
							TG_srv.request.quadrotorState = TRACK;
							TG_srv.request.irobotPosNEDx = irobotPosNED.x;
							TG_srv.request.irobotPosNEDy = irobotPosNED.y;
							TG_srv.request.irobotPosNEDz = irobotPosNED.z;
							TG_srv.request.theta = irobotPosNED.theta;
							if(!TG_client.call(TG_srv))
								ROS_INFO("IARCMission TG_client.call failled......");
							else
								CDJIDrone->local_position_control(TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);	//TODO: YAW = 0?
						}
						else	//irobot is going forward to white/red borders, than APPROACH! 
						{
							iarc_mission::TG TG_srv;
							TG_srv.request.quadrotorState = APPROACH;
							TG_srv.request.irobotPosNEDx = irobotPosNED.x;
							TG_srv.request.irobotPosNEDy = irobotPosNED.y;
							TG_srv.request.irobotPosNEDz = irobotPosNED.z;
							//TG_srv.request.irobotPosNEDz = -6;
							TG_srv.request.theta = irobotPosNED.theta;
							bool gotoApproach = true;
							while(ros::ok() && gotoApproach)
							{
 								ros::spinOnce();
								if(!TG_client.call(TG_srv))
									ROS_INFO("IARCMission TG_client.call failled......");
								else
									CDJIDrone->local_position_control(TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);	//TODO: YAW = 0?
								if(localPosNED.z < 0.2)	//TODO: Z!!??  accumulated error in z axis!!
								{
									ROS_INFO("going to land...");
									mission_land();
									gotoApproach = false;
									CDJIDrone->request_sdk_permission_control();
									mission_takeoff();
									break;
								}
								
							}		
						}	
					}
					else if(localPosNED.z > 1)
					{	// NO irobot, get flight_ctrl_dst in CRUISE mode
						iarc_mission::TG TG_srv;
						TG_srv.request.quadrotorState = CRUISE;
						TG_srv.request.irobotPosNEDx = 0.0;
						TG_srv.request.irobotPosNEDy = 0.0;
						TG_srv.request.irobotPosNEDz = 0.0;
						TG_srv.request.theta = 0.0;
						if(!TG_client.call(TG_srv))
						{
							ROS_INFO("IARCMission TG_client.call failled......");
						}
						else
						{	//TODO: DONOT implement CRUISE MODE temporarily
							CDJIDrone->attitude_control(0x50, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
						}
					}
					loop_rate.sleep();
				}
				break;
			}
			default:
				break;
		}
	}
}
IARCMission::~IARCMission()
{
	ROS_INFO("Destroying IARCMission......");
}

void IARCMission::irobot_pos_callback(const goal_detected::Pose3DConstPtr& msg)
{
	irobotPosNED.x = msg->x;
	irobotPosNED.y = msg->y;
	irobotPosNED.z = msg->z;
	irobotPosNED.theta = msg->theta;
	irobotPosNED.flag = msg->flag;
}

void IARCMission::dji_local_pos_callback(const dji_sdk::LocalPositionConstPtr& msg)
{
	localPosNED.x = msg->x;
	localPosNED.y = msg->y;
	localPosNED.z = msg->z;
}

bool IARCMission::mission_takeoff()
{
	CDJIDrone->drone_arm();
	while((ros::ok()) && (localPosNED.z<1.7))
	{
		ros::spinOnce();
		CDJIDrone->attitude_control(0x80, 0, 0, 0.8, yaw_origin);
		usleep(20000);
	}
	return true;
}

bool IARCMission::mission_land()
{
	for(int i = 0; i < 100; i ++) 
	{
		CDJIDrone->attitude_control(0x80, 0, 0, -0.3, yaw_origin);
		usleep(20000);
		ROS_INFO("landing sleep");
	}
	return true;
}




};
