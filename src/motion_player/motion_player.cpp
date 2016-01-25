#include "ros/ros.h"
#include <trajectory_generation/TrajectoryGeneration.h>
#include <trajectory_generation/Velocity.h>
#include <trajectory_generation/VelocityArray.h>
#include <cstdlib>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <teledrive/Teledrive.h>

using namespace std;


ros::Time current_time;


boost::mutex change_mutex_;
std_msgs::Bool change_; 
void changeCallback(const std_msgs::BoolConstPtr& msg)
{
	boost::mutex::scoped_lock(change_mutex_);
	change_=*msg;
	
	//cout<<"Callback"<<change_.data<<endl;
}

boost::mutex flag_mutex_;
std_msgs::Bool flag_; 
void flagCallback(const std_msgs::BoolConstPtr& msg)
{
	boost::mutex::scoped_lock(flag_mutex_);
	flag_=*msg;
	
}

boost::mutex direction_mutex_;
teledrive::Teledrive direction_; 
void directionCallback(const teledrive::TeledriveConstPtr& msg)
{
	boost::mutex::scoped_lock(direction_mutex_);
	direction_=*msg;
	
	current_time = ros::Time::now();
	cout<<"Twist Come on!!!!!"<<endl;
	
}

float sign(float n)
{
	if(n<0){
		return -1;
	}else if(n>0){
		return 1;
	}else{
		return 0;
	}
}

bool changeCheck(trajectory_generation::VelocityArray v_array, int count)
{
	bool change;
	{
		boost::mutex::scoped_lock(change_mutex_);
		change = change_.data;
	}
	
	//cout<<"Check"<<change<<endl;
	//uint8_t n = v_array.vel.size();
	uint8_t n = 0;
	if(((n > count) && !change)){
		return false;
	}else{
		return true;
	}
}

void commandDecision(	int count,
						teledrive::Teledrive direction, 
						trajectory_generation::VelocityArray v_a,
						trajectory_generation::Velocity& cmd)
{
	float direct_vel = direction.twist.linear.x;
	float direct_ang = direction.twist.angular.z;
	//-----------------Command Publish---------------------//
	//this is for ISMAI
	float hoge = direct_vel*10/0.4;
	int ind = hoge;
	
	for(int i=0; i<v_a.vel.size(); i++){
		cout<<"i="<<i<<"\tlin1 = "<<v_a.vel[i].op_linear<<"\tang1 = "<<v_a.vel[i].op_angular<<endl;
	}
	
	
	if((v_a.vel.size() > count) && (direct_vel>0)){
		cmd.header.stamp = ros::Time::now();
		/*cmd.op_linear = v_a.vel[count].op_linear;
		cmd.op_angular = v_a.vel[count].op_angular;*/
		//this is for ISMAI
		cmd.op_linear = v_a.vel[ind].op_linear;
		cmd.op_angular = v_a.vel[ind].op_angular;
		cout<<"lin1 = "<<cmd.op_linear<<"\tang1 = "<<cmd.op_angular<<endl;
		
		cmd.id = count;
	}
		
	/*------------No path proccess---------------*/
	/*else if(direct_vel<=0.000000001){
		cmd.header.stamp = ros::Time::now();
		//cmd.op_linear = 0.3 * direct_vel;
		//cmd.op_angular = 2 * sign(direct_ang);
		cmd.op_linear = 0;
		cmd.op_angular = direction.op_angular;
		cmd.id = v_a.vel.size()-1;
	}else{
		cmd.op_angular = 0;
		cmd.op_linear = 0;
	}*/
	/*else if(direction.linear.x==0){
		cmd.header.stamp = ros::Time::now();
		cmd.op_linear = 0;
		cmd.op_angular = 0.2 * sign(direction.angular.z);
		cmd.id = v_a.vel.size()-1;
	}*/
}


bool goalCheck(trajectory_generation::VelocityArray v_array, int count)
{
	bool change;
	{
		boost::mutex::scoped_lock(change_mutex_);
		change = change_.data;
	}
	static bool pre_flag;
	bool flag;
	{
		boost::mutex::scoped_lock(flag_mutex_);
		flag = flag_.data;
	}
	//cout<<"Check"<<change<<endl;
	uint8_t n = v_array.vel.size();
	if(((n > count) && !change)){
		pre_flag=flag;
		return false;
	}else{
		pre_flag=flag;
		return true;
	}
}


void motionPlay()
{
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<trajectory_generation::TrajectoryGeneration>("/plan/velocity_call");
	ros::Subscriber sub1 = n.subscribe("/plan/change_flag", 1, changeCallback);
	ros::Subscriber sub4 = n.subscribe("/global_goal/flag", 1, flagCallback);
	ros::Subscriber sub5 = n.subscribe("/plan/direction", 1, directionCallback);
	ros::Publisher pub1 = n.advertise<trajectory_generation::Velocity>("/plan/motion_play", 100);
	ros::Publisher pub2 = n.advertise<std_msgs::Bool>("/plan/pinwheeling", 1);
	
	bool motion_mode;
	trajectory_generation::VelocityArray v_a;
	trajectory_generation::Velocity cmd;
	int count = 0;
	bool pre_flag;
	bool path_yes = true;
	ros::Rate loop_rate(10);
	ros::Time pre_time=ros::Time::now();
	std_msgs::Bool pinwheeling;
	pinwheeling.data = false;
	while(ros::ok()){
		bool flag;
		{
			boost::mutex::scoped_lock(flag_mutex_);
			flag = flag_.data;
		}
		teledrive::Teledrive direction;
		{
			boost::mutex::scoped_lock(direction_mutex_);
			direction=direction_;
		}
		motion_mode = changeCheck(v_a, count);
		cout<<motion_mode<<endl;
		//-----------------Changing Process---------------------//
		if((motion_mode&&(!flag)) || (flag!=pre_flag) ||(current_time!=pre_time)){
			pre_time=current_time;
			trajectory_generation::TrajectoryGeneration trj;
			v_a.vel.clear();
			if (client.call(trj)){
				v_a = trj.response.v_array;
				if(trj.response.tolerance==-1){
					path_yes = false;
				}else{
					path_yes = true;
				}
				ROS_INFO("Changing Trajectory%d",v_a.vel.size());
			}else{
				ROS_ERROR("Failed to call service");
				//return 1;
			}
			count = 0;
		}
		
		commandDecision(count, direction, v_a, cmd);
		pinwheeling.data = !path_yes;
		
		cout<<"lin = "<<cmd.op_linear<<"\tang = "<<cmd.op_angular<<endl;
		pub1.publish(cmd);
		pub2.publish(pinwheeling);
		//-----------------------------------------------------//
		
		pre_flag = flag;
		ros::spinOnce();
		count++;
		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motion_player");
	motionPlay();
	
	ROS_INFO("Killing now!!!!!");
	return 0;
}
