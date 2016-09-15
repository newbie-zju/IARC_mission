#include <ros/ros.h>
#include <IARCMission.h>
#include <math.h>
using namespace std;
namespace mission{
/*
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
						//red					green	------	|------> E theta = PI/2
						//			white						| S theta = PI
						
						//TODO: define the range of theta which means RED, GREEN, WHITE borders
						if((-0.25*M_PI < irobotPosNED.theta) && (irobotPosNED.theta < 0.25*M_PI))	//PI/4 < theta <3PI/4 means irobot is going forward to green borders, than TRACK!	
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
*/
IARCMission::IARCMission(ros::NodeHandle nh):nh_(nh),nh_param("~")
{
	initialize();
	CDJIDrone->request_sdk_permission_control();
	mission_takeoff();
	//CDJIDrone->takeoff();
	quadState = CRUISE;
	ros::Rate loop_rate(50);
	while(ros::ok())
	{
		stateMachine();
		switch(quadState)
		{
			case CRUISE: 
			{
				missionCruise();
				break;				
			}
			case TRACK:
			{
				missionTrack();
				break;
			}
			case APPROACH:
			{
				missionApproach();
				break;
			}
		}
		loop_rate.sleep();
	}

}

IARCMission::~IARCMission()
{
	ROS_INFO("Destroying IARCMission......");
}

// callbacks
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

void IARCMission::boundaryDetect_callback(const geometry_msgs::PointConstPtr& msg)
{
	if((int)msg->z != 0)
		boundaryEmergency = true;
}

void IARCMission::obstacleAvoidance_callback(const std_msgs::Bool msg)
{}


// functions
bool IARCMission::mission_takeoff()
{
/*
	CDJIDrone->drone_arm();
	//CDJIDrone->takeoff();
	while((ros::ok()) && (localPosNED.z<1.7))
	{
		ros::spinOnce();
		ROS_INFO_THROTTLE(0.3, "PosNED.z=%4.2f",localPosNED.z);
		CDJIDrone->attitude_control(0x80, 0, 0, 0.8, yaw_origin);
		ROS_INFO_THROTTLE(1,"taking off...");
		usleep(20000);
	}
*/

	ROS_ERROR("taking off stage 1");
	CDJIDrone->takeoff();
	for(int i = 0; i < 500; i ++) 
	{
		ros::spinOnce();
		ROS_INFO_THROTTLE(0.3,"taking off stage 2");
		CDJIDrone->local_position_control(localPosNED.x, localPosNED.y, 1.8, yaw_origin );
		usleep(20000);
	}

	return true;
}

bool IARCMission::mission_land()
{
	for(int i = 0; i < 100; i ++) 
	{
		CDJIDrone->attitude_control(0x80, 0, 0, -0.3, yaw_origin);
		ROS_INFO_THROTTLE(1,"landing...");
		usleep(20000);
	}
	return true;
}

void IARCMission::initialize()
{
	double yaw_origin_;
	if(!nh_param.getParam("yaw_origin",yaw_origin_))yaw_origin_ = 0.0;
	yaw_origin = (float)yaw_origin_*180.0/M_PI;
	boundaryEmergency = false;
	obstacleEmergency = false;
	quadState = FREE;
	irobot_pos_sub = nh_.subscribe("/goal_detected/goal_pose", 10, &IARCMission::irobot_pos_callback, this);
	dji_local_pos_sub = nh_.subscribe("/dji_sdk/local_position", 10, &IARCMission::dji_local_pos_callback, this);
	TG_client = nh_.serviceClient<iarc_mission::TG>("/TG/TG_service");
	CDJIDrone = new DJIDrone(nh_);
}

bool IARCMission::irobotSafe(double theta)
{
	//bool ret = ((theta > -0.5*M_PI)&&(theta < 0.5*M_PI));
	//ROS_INFO("irobotSafe: theta=%4.2lf,return=%d",theta,(int)ret);
	return ((theta > -0.5*M_PI)&&(theta < 0.5*M_PI));//TODO:this is irobot theta in NED frame, supporse to transform to ground frame
}

int IARCMission::stateMachine()
{
	ros::spinOnce();
	switch(quadState)
	{
		case FREE:
		{
			ROS_INFO_THROTTLE(1,"FREE...");
			break;
		}
		case CRUISE:
		{
			if(gotoCruise())
			{
				ROS_INFO_THROTTLE(0.2,"CRUISE->CRUISE");
				quadState = CRUISE;
				break;
			}
			if(gotoTrack())
			{
				ROS_INFO_THROTTLE(0.2,"CRUISE->TRACK");
				quadState = TRACK;
				break;
			}
			break;
		}
		case TRACK:
		{
			if(gotoCruise())
			{
				ROS_INFO_THROTTLE(0.2,"TRACK->CRUISE");
				quadState = CRUISE;
				break;
			}
			if(gotoApproach())
			{
				ROS_INFO_THROTTLE(0.2,"TRACK->APPROACH");
				quadState = APPROACH;
				break;
			}	// ATTENTION: to determin whether to APPROACH before TRACK
			if(gotoTrack())
			{
				ROS_INFO_THROTTLE(0.2,"TRACK->TRACK");
				quadState = TRACK;
				break;
			}
			break;
		}
		case APPROACH:
		{
			ROS_INFO_THROTTLE(0.2,"APPROACH->");
			break;
		}
	}
}

void IARCMission::missionCruise()
{
	ros::spinOnce();
	iarc_mission::TG TG_srv;
	TG_srv.request.quadrotorState = CRUISE;
	TG_srv.request.irobotPosNEDx = 0.0;
	TG_srv.request.irobotPosNEDy = 0.0;
	TG_srv.request.irobotPosNEDz = 0.0;
	TG_srv.request.theta = 0.0;
	TG_srv.request.cruiseStep = 0;
	if(!TG_client.call(TG_srv))
		ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
	else
	{
		if((uint8_t)0x90 == TG_srv.response.flightFlag)
			CDJIDrone->local_position_control(TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
		if((uint8_t)0x50 == TG_srv.response.flightFlag)
			CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
	}
}

void IARCMission::missionTrack()
{
	ros::spinOnce();
	iarc_mission::TG TG_srv;
	TG_srv.request.quadrotorState = TRACK;
	TG_srv.request.irobotPosNEDx = irobotPosNED.x;
	TG_srv.request.irobotPosNEDy = irobotPosNED.y;
	TG_srv.request.irobotPosNEDz = irobotPosNED.z;
	TG_srv.request.theta = irobotPosNED.theta;
	TG_srv.request.cruiseStep = 0;
	if(!TG_client.call(TG_srv))
		ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
	else
	{
		if((uint8_t)0x90 == TG_srv.response.flightFlag)
			CDJIDrone->local_position_control(TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
		if((uint8_t)0x50 == TG_srv.response.flightFlag)
			CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
	}
}

void IARCMission::missionApproach()
{
	iarc_mission::TG TG_srv;
	TG_srv.request.quadrotorState = APPROACH;
	TG_srv.request.irobotPosNEDx = irobotPosNED.x;
	TG_srv.request.irobotPosNEDy = irobotPosNED.y;
	TG_srv.request.irobotPosNEDz = irobotPosNED.z;
	//TG_srv.request.irobotPosNEDz = -6;
	TG_srv.request.theta = irobotPosNED.theta;
	TG_srv.request.cruiseStep = 0;
	bool gotoApproach = true;
	while(ros::ok() && gotoApproach)
	{
		ros::spinOnce();
		if(!TG_client.call(TG_srv))
			ROS_INFO("IARCMission TG_client.call failled......");
		else
		{
			ROS_INFO_THROTTLE(0.3, "mission_APP:%d,%3.1f,%3.1f,%3.1f",TG_srv.response.flightFlag,TG_srv.response.flightCtrlDstx,TG_srv.response.flightCtrlDsty,TG_srv.response.flightCtrlDstz);
			if((uint8_t)0x90 == TG_srv.response.flightFlag)
				CDJIDrone->local_position_control(TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
			if((uint8_t)0x50 == TG_srv.response.flightFlag)
				CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
		}
		if(localPosNED.z < 0.2)	//TODO: Z!!??  accumulated error in z axis!!
		{
			ROS_INFO("going to land...");
			mission_land();
			gotoApproach = false;
			//CDJIDrone->request_sdk_permission_control();
			mission_takeoff();
			break;
		}
	}
	quadState = CRUISE;
}

bool IARCMission::gotoCruise()
{
	if(quadState == CRUISE)
		return (!(irobotPosNED.flag>0) || ((irobotPosNED.flag>0)&&(irobotSafe(irobotPosNED.theta))));
	if(quadState == TRACK)
		return (!(irobotPosNED.flag>0) || ((irobotPosNED.flag>0)&&(irobotSafe(irobotPosNED.theta))));	//TODO: not consider loosing irobot while tarcking
	else return false;
}

bool IARCMission::gotoTrack()
{
	if(quadState == CRUISE)
		return (irobotPosNED.flag>0)&&(!irobotSafe(irobotPosNED.theta));	//TODO:only if irobot is not safe,then goto track?
	if(quadState == TRACK)
		return (irobotPosNED.flag>0)&&(!irobotSafe(irobotPosNED.theta));
	else return false;
}

bool IARCMission::gotoApproach()
{
	if(getLength2f(localPosNED.x-irobotPosNED.x,localPosNED.y-irobotPosNED.y)<1.0) return true;
	else return false;
}

float IARCMission::getLength2f(float x, float y)
{
	return sqrt(x*x+y*y);
}







};
