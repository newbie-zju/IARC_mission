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
	//for(int i = 0; i < 300; i ++) 
	//{
	//	ros::spinOnce();
	//	ROS_INFO_THROTTLE(0.3,"taking off stage 2");
	//	CDJIDrone->local_position_control(localPosNED.x, localPosNED.y, 1.8, yaw_origin );
	//	usleep(20000);
	//}
	quadState = CRUISE;
	ros::Rate loop_rate(50);
	while(ros::ok())
	{
		stateMachine();
		switch(quadState)
		{
			case FREE:
			{
				missionFree();
				break;
			}
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


void IARCMission::initialize()
{
	double yaw_origin_;
	if(!nh_param.getParam("yaw_origin",yaw_origin_))yaw_origin_ = 0.0;
	yaw_origin = (float)yaw_origin_*180.0/M_PI;
	boundaryEmergency = false;
	obstacleEmergency = false;
	quadState = FREE;
	free_time = ros::Time::now();
	free_time_prev = free_time;
	irobot_pos_sub = nh_.subscribe("/goal_detected/goal_pose", 10, &IARCMission::irobot_pos_callback, this);
	dji_local_pos_sub = nh_.subscribe("/dji_sdk/local_position", 10, &IARCMission::dji_local_pos_callback, this);
	TG_client = nh_.serviceClient<iarc_mission::TG>("/TG/TG_service");
	CDJIDrone = new DJIDrone(nh_);
}


int IARCMission::stateMachine()
{
	ros::spinOnce();
	switch(quadState)
	{
		case FREE:
		{
			//ROS_ERROR("SM: FREE");
			if(gotoFree())
			{
				ROS_INFO_THROTTLE(0.2,"FREE->FREE");
				quadState = FREE;
				break;
			}
			if(gotoCruise())
			{
				ROS_INFO_THROTTLE(0.2, "FREE->CRUISE");
				quadState = CRUISE;
				break;
			}
			if(gotoTrack())
			{
				ROS_INFO_THROTTLE(0.2, "FREE->TRACK");
				quadState = TRACK;
				break;
			}
			break;
		}
		case CRUISE:
		{
			//ROS_ERROR("SM: CRUISE");
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
			//ROS_ERROR("SM: TRACK");
			if(gotoFree())
			{
				ROS_INFO_THROTTLE(0.2, "TRACK->FREE");
				quadState = FREE;
				free_time = ros::Time::now();
				free_time_prev = ros::Time::now();
				break;
			}
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
// 		if((uint8_t)0x90 == TG_srv.response.flightFlag)
// 			CDJIDrone->local_position_control(TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
// 		if((uint8_t)0x40 == TG_srv.response.flightFlag)
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
	while(ros::ok())
	{
		if(!TG_client.call(TG_srv))
			ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
		else
		{
			if((uint8_t)0x80 == TG_srv.response.flightFlag)
			{
				if(getLength2f(TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty) < 0.3)break;
				CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
			if((uint8_t)0x40 == TG_srv.response.flightFlag)
				CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
		}
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
			sleep(10);
			mission_takeoff();
			break;
		}
	}
	quadState = FREE;
	free_time = ros::Time::now();
	free_time_prev = ros::Time::now();
	
}

void IARCMission::missionFree()
{
	free_time = ros::Time::now();
	freeTimer = free_time - free_time_prev;
	ROS_ERROR_THROTTLE(0.2,"missionFree: freeTimer = %4.2f",(float)freeTimer.toSec());
	iarc_mission::TG TG_srv;
	TG_srv.request.quadrotorState = FREE;
	TG_srv.request.irobotPosNEDx = 0.0;
	TG_srv.request.irobotPosNEDy = 0.0;
	TG_srv.request.irobotPosNEDz = 0.0;
	TG_srv.request.theta = irobotPosNED.theta;
	TG_srv.request.cruiseStep = 0;
	if(!TG_client.call(TG_srv))
		ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
	else
	{
		if((uint8_t)0x80 == TG_srv.response.flightFlag)
			CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
		if((uint8_t)0x40 == TG_srv.response.flightFlag)
			CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
	}
}


bool mission::IARCMission::gotoFree()
{
	if(quadState == FREE)
	{
		bool ret =  ((!((float)freeTimer.toSec() > 1.0)) && !gotoTrack());
		ROS_INFO_THROTTLE(0.2,"gotoFree: freeTimer=%4.2f,ret=%d",(float)freeTimer.toSec(),(int)ret);
		return ret;
	}
	if(quadState == TRACK)
		return (!(irobotPosNED.flag>0) || ((irobotPosNED.flag>0)&&(irobotSafe(irobotPosNED.theta))));
}


bool IARCMission::gotoCruise()
{
	if(quadState == FREE)
		return (((float)freeTimer.toSec() > 1.0) && !gotoTrack());
	if(quadState == CRUISE)
		return (!(irobotPosNED.flag>0) || ((irobotPosNED.flag>0)&&(irobotSafe(irobotPosNED.theta))));
	if(quadState == TRACK)
		return (!(irobotPosNED.flag>0) || ((irobotPosNED.flag>0)&&(irobotSafe(irobotPosNED.theta))));	//TODO: not consider loosing irobot while tarcking
	else return false;
}

bool IARCMission::gotoTrack()
{
	if(quadState == FREE)
	{
		//ROS_INFO("gotoTrack");
		return (irobotPosNED.flag>0)&&(!irobotSafe(irobotPosNED.theta));
	}
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

bool IARCMission::irobotSafe(double theta)
{
	//bool ret = ((theta > -0.5*M_PI)&&(theta < 0.5*M_PI));
	//ROS_INFO("irobotSafe: theta=%4.2lf,return=%d",theta,(int)ret);
	//ROS_ERROR("%4.2f<theta(%4.2f)<%4.2f",yaw_origin*M_PI/180.0 - 0.5*M_PI,theta,yaw_origin*M_PI/180.0 + 0.5*M_PI);
    return ((theta > yaw_origin*M_PI/180.0 - 0.5*M_PI+M_PI)&&(theta < yaw_origin*M_PI/180.0 + 0.5*M_PI+M_PI));//TODO:this is irobot theta in NED frame, supporse to transform to ground frame
}

bool IARCMission::mission_takeoff()
{
	ros::spinOnce();
	CDJIDrone->drone_arm();
	while((ros::ok()) && (localPosNED.z<1.2))
	{
		ros::spinOnce();
		//ROS_INFO_THROTTLE(0.3, "PosNED.z=%4.2f",localPosNED.z);
		CDJIDrone->attitude_control(0x00, 0, 0, 0.8, yaw_origin);
		ROS_INFO_THROTTLE(1,"taking off...");
		usleep(20000);
	}
/*
	CDJIDrone->takeoff();
	for(int i = 0; i < 500; i ++) 
	{
		ros::spinOnce();
		ROS_INFO_THROTTLE(0.3,"taking off stage 2");
		CDJIDrone->local_position_control(localPosNED.x, localPosNED.y, 1.6, yaw_origin );
		usleep(20000);
	}
*/
	
	return true;
}

bool IARCMission::mission_land()
{

	for(int i = 0; i < 100; i ++) 
	{
		CDJIDrone->attitude_control(0x40, 0, 0, -0.3, yaw_origin);
		ROS_INFO_THROTTLE(1,"landing...");
		usleep(20000);
	}

	//CDJIDrone->landing();
	return true;
}

float IARCMission::getLength2f(float x, float y)
{
	return sqrt(x*x+y*y);
}







};
