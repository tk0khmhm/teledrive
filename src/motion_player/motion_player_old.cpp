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


using namespace std;

boost::mutex goal_mutex_;
geometry_msgs::PoseStamped goal_; 
void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	boost::mutex::scoped_lock(goal_mutex_); // mutexロック スコープの範囲だけ有効
	goal_=*msg;  // コピー
	
}

boost::mutex odo_mutex_;
nav_msgs::Odometry _odo; 
void tinyCallback(const nav_msgs::OdometryConstPtr& msg)
{
	boost::mutex::scoped_lock(odo_mutex_); 
	_odo=*msg; 
}

bool changeCheck(trajectory_generation::VelocityArray v_array, int count)
{
	uint8_t n = v_array.vel.size();
	if(n>count)	return false;
	else		return true;
	
}

void motionPlay()
{
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<trajectory_generation::TrajectoryGeneration>("/plan/local_path");
	//ros::Subscriber sub1 = n.subscribe("/move_base_simple/goal", 100, goalCallback);
	ros::Subscriber sub1 = n.subscribe("/local_goal/dr", 100, goalCallback);
	ros::Subscriber sub2 = n.subscribe("tinypower/odom", 100, tinyCallback);
	ros::Publisher pub1 = n.advertise<trajectory_generation::Velocity>("tinypower/command_velocity", 100);
	
	tf::TransformListener listener;
	trajectory_generation::TrajectoryGeneration trj;
	trj.request.params.v0 = 0.3;
	trj.request.params.a0 = 0.4;
	trj.request.params.vt = 0.6;
	trj.request.params.af = -0.4;
	trj.request.params.vf = 0;
	trj.request.params.k0 = 0;
	
	bool motion_mode;
	trajectory_generation::VelocityArray v_a;
	trajectory_generation::Velocity cmd;
	int count = 0;
	ros::Rate loop_rate(10);
	while(ros::ok()){
		geometry_msgs::PoseStamped goal_r;
		{
			boost::mutex::scoped_lock(goal_mutex_); // mutexロック スコープの範囲だけ有効
			goal_r=goal_;
		}
		nav_msgs::Odometry odo; 
		{
			boost::mutex::scoped_lock(odo_mutex_); 
			odo = _odo;
			trj.request.params.v0 = odo.twist.twist.linear.x;
			trj.request.params.k0 = odo.twist.twist.angular.z/(_odo.twist.twist.linear.x+0.0000000001);
			//if(fabs(trj.request.params.v0)<0.1)	trj.request.params.v0 = 0.1;
			//if(fabs(trj.request.params.k0)<0.01)	trj.request.params.k0 = 0.01;
		}
		
		tf::StampedTransform transform;
		try{
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		try{
			listener.transformPose("/base_link",goal_r.header.stamp,goal_r,"/map",trj.request.goal);
		}catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		trj.request.start.pose.position.x = transform.getOrigin().x();
		trj.request.start.pose.position.y = transform.getOrigin().y();
		trj.request.start.pose.position.z = 0;
		float angle = tf::getYaw(transform.getRotation());
		trj.request.start.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,angle);
		
		motion_mode = changeCheck(v_a, count);
		//if(motion_mode){
			//v_a.vel.clear();
			if (client.call(trj)){
				v_a = trj.response.v_array;
				ROS_INFO("Changing Trajectory");
			}else{
				ROS_ERROR("Failed to call service");
				//return 1;
			}
			count = 0;
		//}
		cmd.header.stamp = ros::Time::now();
		cmd.op_linear = v_a.vel[count].op_linear;
		cmd.op_angular = -v_a.vel[count].op_angular;
		cout<<"lin = "<<cmd.op_linear<<"\tang = "<<cmd.op_angular<<endl;
		pub1.publish(cmd);
		count++;
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motion_player");
	motionPlay();
	return 0;
}
