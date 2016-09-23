#include <IARCMission.h>
#include <math.h>
using namespace std;
namespace mission{
/*
VERTICAL_VELOCITY = 0x00,
VERTICAL_POSITION = 0x10,
VERTICAL_THRUST = 0x20,
HORIZONTAL_ANGLE = 0x00,
HORIZONTAL_VELOCITY = 0x40,
HORIZONTAL_POSITION = 0X80
*/
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
	sleep(4);
	mission_takeoff();
/*
	ros::spinOnce();
	while(ros::ok() && quadrotorGroundPos.x < 2.0)
	{
		ros::spinOnce();
		float k = 0.8;
		float tarVz = k * (1.6 - localPosNED.z);
		iarc_tf::Velocity srv;
		srv.request.velocityFrame = GROUND;
		srv.request.velocityX = 0.3;
		srv.request.velocityY = 0;
		float goinsideVx = 0.0;
		float goinsideVy = 0.0;
		//ROS_ERROR("tarVx = %f",tarVx);
		if(tf_vel_client.call(srv))
		{
			goinsideVx = srv.response.velocityXRes;
			goinsideVx = srv.response.velocityYRes;
		}
		else{ROS_ERROR_THROTTLE(0.2,"NO TF SERVICE!");}
		CDJIDrone->attitude_control(0x40, goinsideVx, goinsideVx, tarVz, yaw_origin);
		ROS_INFO_THROTTLE(0.2,"GO INSIDE...");
	}
*/
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
	irobotsPosNEDWithReward.x.clear();
	irobotsPosNEDWithReward.y.clear();
	irobotsPosNEDWithReward.z.clear();
	irobotsPosNEDWithReward.theta.clear();
	irobotsPosNEDWithReward.flag = 0;
	irobotsPosNEDWithReward.reward.clear();
	irobotPosNED.x = 0.0;
	irobotPosNED.y = 0.0;
	irobotPosNED.z = 0.0;
	irobotPosNED.flag = 0;
	irobotPosNED.theta = 0.0;
	if(msg->flag > 0)
	{
		irobotsPosNEDWithReward.flag = msg->flag;
		irobotPosNED.flag = msg->flag;
		for(int i = 0;i != msg->flag;i++)
		{
			irobotsPosNEDWithReward.x.push_back(msg->x[i]);
			irobotsPosNEDWithReward.y.push_back(msg->y[i]);
			irobotsPosNEDWithReward.z.push_back(msg->z[i]);
			irobotsPosNEDWithReward.theta.push_back(msg->theta[i]);
			irobotsPosNEDWithReward.reward.push_back(0.0);
		}
		irobotReward();		//calculate reward of each irobot
		float minReward = 999.9;
		int minRewardIndex = 999;
		for(int i = 0;i != irobotsPosNEDWithReward.flag;i++)
		{
			if(irobotsPosNEDWithReward.reward[i] < minReward)	//find the irobot with the smallest reward, which meas forward to red line, and should to be track & approach
			{
				minReward = irobotsPosNEDWithReward.reward[i];
				minRewardIndex = i;
			}
		}
		irobotPosNED.x = irobotsPosNEDWithReward.x[minRewardIndex];
		irobotPosNED.y = irobotsPosNEDWithReward.y[minRewardIndex];
		irobotPosNED.z = irobotsPosNEDWithReward.z[minRewardIndex];
		irobotPosNED.theta = irobotsPosNEDWithReward.theta[minRewardIndex];
		ROS_ERROR_THROTTLE(0.2,"selected irobotPosNED=(%4.2f,%4.2f,%4.2f),flag=%d",irobotPosNED.x,irobotPosNED.y,irobotPosNED.theta,irobotPosNED.flag);
	}
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

void IARCMission::quadrotorPosGroundCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
	quadrotorGroundPos.x = msg->point.x;
	quadrotorGroundPos.y = msg->point.y;
}
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
	irobotPosNED.x = 0.0;
	irobotPosNED.y = 0.0;
	irobotPosNED.z = 0.0;
	irobotPosNED.theta = 0.0;
	irobotPosNED.flag = 0;
	irobotsPosNEDWithReward.x.clear();
	irobotsPosNEDWithReward.y.clear();
	irobotsPosNEDWithReward.z.clear();
	irobotsPosNEDWithReward.theta.clear();
	irobotsPosNEDWithReward.flag = 0;
	irobotsPosNEDWithReward.reward.clear();
	irobot_pos_sub = nh_.subscribe("/goal_detected/goal_pose", 10, &IARCMission::irobot_pos_callback, this);
	dji_local_pos_sub = nh_.subscribe("/dji_sdk/local_position", 10, &IARCMission::dji_local_pos_callback, this);
	quadrotorPosGround_sub = nh_.subscribe("/ground_position",10, &IARCMission::quadrotorPosGroundCallback,this);
	TG_client = nh_.serviceClient<iarc_mission::TG>("/TG/TG_service");
	tf_vel_client = nh_.serviceClient<iarc_tf::Velocity>("ned_world_velocity_transform_srvice");
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
			if((uint8_t)0x80 == TG_srv.response.flightFlag)
				//CDJIDrone->local_position_control(TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
				CDJIDrone->attitude_control(0x80, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
			if((uint8_t)0x50 == TG_srv.response.flightFlag)
			{
				ros::spinOnce();
				while((ros::ok()) && (localPosNED.z<1.4))
				{
					ros::spinOnce();
					//ROS_INFO_THROTTLE(0.3, "PosNED.z=%4.2f",localPosNED.z);
					CDJIDrone->attitude_control(0x40, 0, 0, 0.8, yaw_origin);
					ROS_INFO_THROTTLE(1,"avoidance taking off...");
					usleep(20000);
				}
				break;
			}
				//if has obstacle while approaching , break, will FREE
				//CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
		}
		if(localPosNED.z < 0.3)	//TODO: Z!!??  accumulated error in z axis!!
		{
			ROS_INFO("going to land...");
			mission_land();
			gotoApproach = false;
			//CDJIDrone->request_sdk_permission_control();
			//sleep(10);
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
	TG_srv.request.theta = 0.0;
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


bool IARCMission::gotoFree()
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
	return ((irobotPosNED.flag>0) && (!irobotSafe(irobotPosNED.theta)) && (getLength2f(localPosNED.x-irobotPosNED.x,localPosNED.y-irobotPosNED.y)<1.0));
}

bool IARCMission::irobotSafe(double theta)
{
						//			white												^  Gx theta = yaw_origin
						//green					red				yaw_origin - 0.5*M_PI	|
						//green					red								------	|------> Gy 
						//			white												| 
	float dtheta = theta - yaw_origin*M_PI/180.0;
	ROS_ERROR("theta=%4.2f,yaw_0=%4.2f,dtheta=%4.2f,ret=%d",theta,yaw_origin*M_PI/180.0,dtheta,(int)(limitAng(dtheta)<0.0));
	return (limitAng(dtheta)>0.0);
/*bool ret = ((theta > limitAng(yaw_origin*M_PI/180.0 - 0.5*M_PI - 0.5*M_PI))&&(theta < limitAng(yaw_origin*M_PI/180.0 - 0.5*M_PI + 0.5*M_PI)));
	ROS_ERROR("%4.2f < theta(%4.2f) < %4.2f,ret=%d",limitAng(yaw_origin*M_PI/180.0 - 0.5*M_PI - 0.5*M_PI),theta,limitAng(yaw_origin*M_PI/180.0 - 0.5*M_PI + 0.5*M_PI),(int)ret);
    return ret;//TODO:this is irobot theta in NED frame, supporse to transform to ground frame
*/
}

void IARCMission::irobotReward()
{
	//if theta is forward to green line, reward is small, should goto track and approach, should be selected
	for(int i = 0;i != irobotsPosNEDWithReward.flag;i++)
	{
		float reward;
		//reward = fabs(limitAng(irobotsPosNEDWithReward.theta[i]) - limitAng(yaw_origin*M_PI/180.0 + 0.5*M_PI));
		reward  =fabs(limitAng(irobotsPosNEDWithReward.theta[i] - limitAng(yaw_origin*M_PI/180.0 - 0.5*M_PI)));
		//ROS_INFO_THROTTLE(0.2,"irobotPosNED=(%4.2f,%4.2f),reward=%4.2f",irobotsPosNEDWithReward.x[i],irobotsPosNEDWithReward.y[i],reward);
		irobotsPosNEDWithReward.reward.push_back(reward);
	}
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

	for(int i = 0; i < 200; i ++) 
	{
		CDJIDrone->attitude_control(0x00, 0, 0, -0.6, yaw_origin);
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

IARCMission::irobotPose IARCMission::predictIrobotPose(IARCMission::irobotPose irobotpose, float TPred, float TInLoop)
{
	float dT = 0.1;
	IARCMission::irobotPose ret;
	for(int i = 1;i < floor(TPred/dT);i++)
	{
		if(fmod(dT*i + TInLoop, 20)<2.0)
		{
			ret.x = irobotpose.x;
			ret.y = irobotpose.y;
			ret.theta = M_PI/2.0*dT + irobotpose.theta;
		}
		else
		{
			ret.x = irobotpose.x + 0.3333333*cos(irobotpose.theta)*dT;
			ret.y = irobotpose.y + 0.3333333*sin(irobotpose.theta)*dT;
			ret.theta = M_PI/2.0*dT + irobotpose.theta;			
		}
	}
	return ret;
}

float IARCMission::limitAng(float theta)
{
	if(theta < -M_PI)theta += 2*M_PI;
	if(theta > M_PI)theta -= 2*M_PI;
	return theta;
}






};
