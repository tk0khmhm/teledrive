#include "ros/ros.h"
#include <trajectory_generation/TrajectoryGeneration.h>
#include <iostream>
#include <rwrc13_msgs/Velocity.h>
#include <std_msgs/Bool.h>
#include <trajectory_generation/Velocity.h>
#include <teledrive/Teledrive.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
using namespace std;


boost::mutex flag_mutex_;
std_msgs::Bool flag_; 
void flagCallback(const std_msgs::BoolConstPtr& msg)
{
	boost::mutex::scoped_lock(flag_mutex_);
	flag_=*msg;
	
}

boost::mutex emgu_mutex_;
std_msgs::Bool emgu_; 
void emguCallback(const std_msgs::BoolConstPtr& msg)
{
	boost::mutex::scoped_lock(emgu_mutex_);
	emgu_=*msg;	
}

boost::mutex direction_mutex_;
teledrive::Teledrive direction_; 
void directionCallback(const teledrive::TeledriveConstPtr& msg)
{
	boost::mutex::scoped_lock(direction_mutex_);
	direction_=*msg;
}

float d2a(float d_angle)
{
	float cmd_angle;
	if(fabs(d_angle) > M_PI_2){
		d_angle = M_PI_2 - (fabs(d_angle) - M_PI_2);
	}
	
	cmd_angle = d_angle/5;
	return cmd_angle;
}


boost::mutex v_pub_mutex_;
ros::Publisher _v_pub;
void vCallback(const trajectory_generation::VelocityConstPtr& v)
{
	ros::Publisher pub;
	{
		boost::mutex::scoped_lock(v_pub_mutex_);	
		pub = _v_pub;
	}
	teledrive::Teledrive direction;
	{
		boost::mutex::scoped_lock(direction_mutex_);
		direction=direction_;
	}
	bool pinwheeling;
	{
		boost::mutex::scoped_lock(flag_mutex_);
		pinwheeling = flag_.data;
	}
	bool u_stop;
	{
		boost::mutex::scoped_lock(emgu_mutex_);
		u_stop = emgu_.data;
	}

	
	
	
	
	float direct_vel = direction.twist.linear.x;
	float direct_ang = direction.twist.angular.z;
	
	rwrc13_msgs::Velocity vel;
	vel.op_linear 	= v->op_linear;
	vel.op_angular 	= v->op_linear*v->op_angular *(-1);
	
	if( (pinwheeling)
		|| (u_stop && (vel.op_linear>0.001)) ){
		vel.op_linear = 0;
//		vel.op_angular = -d2a(direct_ang);
		vel.op_angular = -direction.op_angular;
		cout<<"pinwheeling"<<endl;
	}
	
	if(direct_vel<=0.000000001){
		vel.op_linear = 0;
		vel.op_angular = -direction.op_angular;
		cout<<"rolling"<<endl;
	}

//	vel.op_linear = direct_vel*0.1;
//	vel.op_angular = direct_ang*0.1;

	vel.op_linear = vel.op_linear*1;
	//vel.op_angular = vel.op_angular*1;	// 0号機
	vel.op_angular = -vel.op_angular*1;	// 1号機


	pub.publish(vel);
	printf("vel.op_linnear = \t %2.3f \n",vel.op_linear);
	printf("vel.op_angular = \t %2.3f \n\n",vel.op_angular);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "control_infant");
	ros::NodeHandle n;
	ros::Publisher pub1 = n.advertise<rwrc13_msgs::Velocity>("tinypower/command_velocity", 1);
	ros::Subscriber sub1 = n.subscribe("/plan/motion_play", 100, vCallback);
	ros::Subscriber sub2 = n.subscribe("/plan/direction", 1, directionCallback);
	ros::Subscriber sub3 = n.subscribe("/plan/pinwheeling", 1, flagCallback);
	ros::Subscriber sub4 = n.subscribe("/plan/urg_stop", 1, emguCallback);
//	ros::Subscriber sub5 = n.subscribe("/plan/back_stop", 1, emgbCallback);
	{
		boost::mutex::scoped_lock(v_pub_mutex_);	
		_v_pub = pub1;
	}
	ros::spin();
	
	
}

