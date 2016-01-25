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
//---------------From library---------------//
#include "trajectory_generation/Visualize_lib.h"


boost::mutex trj_mutex_;
trajectory_generation::TrajectoryGeneration::Response path_v;

boost::mutex path_pub_mutex_;
ros::Publisher _path_pub;
boost::mutex va_pub_mutex_;
ros::Publisher _va_pub;
using namespace std;

/////////////////////////////////////////////////////////
//-------------------CallBack!!!!!---------------------//
/////////////////////////////////////////////////////////
boost::mutex goal_mutex_;
geometry_msgs::PoseStamped goal_; 
void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	boost::mutex::scoped_lock(goal_mutex_);
	goal_=*msg;
}

boost::mutex glo_goal_mutex_;
geometry_msgs::PoseStamped glo_goal_; 
void globalGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	boost::mutex::scoped_lock(glo_goal_mutex_);
	glo_goal_=*msg;
}

boost::mutex odo_mutex_;
nav_msgs::Odometry _odo; 
void tinyCallback(const nav_msgs::OdometryConstPtr& msg)
{
	boost::mutex::scoped_lock(odo_mutex_); 
	_odo=*msg; 
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
	cout<<"Twist Come on!!!!!"<<endl;
	
}
/////////////////////////////////////////////////////////
//-------------------Visualize!!!!!--------------------//
/////////////////////////////////////////////////////////
void showPath(nav_msgs::Path path)
{
	std_msgs::ColorRGBA rgba_in;
	rgba_in.r = 1.0;
	rgba_in.g = 0;
	rgba_in.b = 0;
	rgba_in.a = 0.8;
	//Visualization* marker_line = new Visualization(rgba_in, ADD,0.1, "OnlyPath","/odom");
	Visualization* marker_line = new Visualization(rgba_in, ADD,0.1, "OnlyPath","/tiny_odom");
	ros::Publisher pub;
	{
		boost::mutex::scoped_lock(path_pub_mutex_);	
		pub = _path_pub;
	}
	
	visualization_msgs::Marker line1;
	marker_line->convertPath2MarkerLine(path, line1, 1);
		
	pub.publish(line1);
	delete marker_line;
}

void velPub(trajectory_generation::VelocityArray v_array)
{
	ros::Publisher pub;
	{
		boost::mutex::scoped_lock(va_pub_mutex_);	
		pub = _va_pub;
	}
	
	pub.publish(v_array);
}

/////////////////////////////////////////////////////////
//---------------------Server!!!!!---------------------//
/////////////////////////////////////////////////////////
bool server(trajectory_generation::TrajectoryGeneration::Request  &req,
         trajectory_generation::TrajectoryGeneration::Response &res )
{
	boost::mutex::scoped_lock(trj_mutex_);
	res = path_v;
	showPath(res.path);
	velPub(res.v_array);
	return true;
}

void trjSetMode(trajectory_generation::TrajectoryGeneration& trj)
{
	geometry_msgs::PoseStamped glo_goal;
	{
		boost::mutex::scoped_lock(glo_goal_mutex_); // mutexロック スコープの範囲だけ有効
		glo_goal=glo_goal_;
	}
	bool flag;
	{
		boost::mutex::scoped_lock(flag_mutex_);
		flag = flag_.data;
	}
	teledrive::Teledrive direction;
	{
		boost::mutex::scoped_lock(direction_mutex_); // mutexロック スコープの範囲だけ有効
		direction=direction_;
	}
	
	if(!flag){
		trj.request.fin = false;
		/*trj.request.params.vt = 0.6;
		trj.request.params.af = -0.4;
		trj.request.params.vf = 0.0;*/
		trj.request.params.vt = direction.twist.linear.x;
		trj.request.params.af = -0.4;
		trj.request.params.vf = direction.twist.linear.x;
		trj.request.goal.pose.position.x = direction.twist.angular.z;
	}else{
		trj.request.fin = true;
		trj.request.params.a0 = -0.1;
		trj.request.params.vt = 0.3;
		trj.request.params.af = -0.1;
		trj.request.params.vf = 0.0;
		trj.request.goal = glo_goal;
		//trj.request.goal.header.stamp = goal_r.header.stamp;
	}
}

void trajectoryManage()
{
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<trajectory_generation::TrajectoryGeneration>("/plan/local_path");
	ros::ServiceServer service = n.advertiseService("/plan/velocity_call", server);
	//ros::Subscriber sub1 = n.subscribe("/move_base_simple/goal", 100, goalCallback);
	ros::Subscriber sub1 = n.subscribe("/local_goal/dr", 100, goalCallback);
	ros::Subscriber sub2 = n.subscribe("/tinypower/odom", 100, tinyCallback);
	ros::Subscriber sub3 = n.subscribe("/global_goal", 100, globalGoalCallback);
	ros::Subscriber sub4 = n.subscribe("/global_goal/flag", 1, flagCallback);
	ros::Subscriber sub5 = n.subscribe("/plan/direction", 1, directionCallback);
	//ros::Publisher pub1 = n.advertise<trajectory_generation::Velocity>("tinypower/command_velocity", 100);
	ros::Publisher pub1 = n.advertise<visualization_msgs::Marker>("plan/localpath_vis", 100);
	ros::Publisher pub2 = n.advertise<trajectory_generation::VelocityArray>("plan/velocity_array", 100);
	{
		boost::mutex::scoped_lock(path_pub_mutex_);	
		_path_pub = pub1;
	}
	{
		boost::mutex::scoped_lock(va_pub_mutex_);	
		_va_pub = pub2;
	}
	
	tf::TransformListener listener;
	trajectory_generation::TrajectoryGeneration trj;
	trj.request.params.v0 = 0.3;
	trj.request.params.a0 = 0.4;
	trj.request.params.vt = 0.6;
	trj.request.params.af = -0.4;
	trj.request.params.vf = 0;
	trj.request.params.k0 = 0;
	
	trajectory_generation::VelocityArray v_a;
	trajectory_generation::Velocity cmd;
	ros::Rate loop_rate(10);
	while(ros::ok()){
		nav_msgs::Odometry odo; 
		{
			boost::mutex::scoped_lock(odo_mutex_); 
			odo = _odo;
			trj.request.params.v0 = odo.twist.twist.linear.x;
			trj.request.params.k0 = odo.twist.twist.angular.z/(_odo.twist.twist.linear.x+0.0000000001);
		}
		
		tf::StampedTransform transform;
		try{
			//listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
			listener.lookupTransform("/tiny_odom", "/base_link", ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		
		trj.request.start.pose.position.x = transform.getOrigin().x();
		trj.request.start.pose.position.y = transform.getOrigin().y();
		trj.request.start.pose.position.z = 0;
		float angle = tf::getYaw(transform.getRotation());
		trj.request.start.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,angle);
		trjSetMode(trj);
		
		v_a.vel.clear();
		//if(trj.request.params.vt > 0){
		if(true){	// 20160118 変更
			if (client.call(trj)){
				v_a = trj.response.v_array;
				ROS_INFO("Changing Trajectory");
			}else{
				ROS_ERROR("Failed to call service");
				//return 1;
			}
		}else{
			cout<<"else"<<endl;
		}
		
		{
			boost::mutex::scoped_lock(trj_mutex_);
			path_v = trj.response;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_manager");
	trajectoryManage();
	return 0;
}
