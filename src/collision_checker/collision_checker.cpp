#include "ros/ros.h"
#include <trajectory_generation/TrajectoryGeneration.h>
#include <trajectory_generation/Velocity.h>
#include <trajectory_generation/VelocityArray.h>
#include <cstdlib>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

//---------------From library---------------//
#include "trajectory_generation/Visualize_lib.h"
#include "trajectory_generation/LocalPlanning_lib.h"


const float dt = 0.1;

boost::mutex path_pub_mutex_;
ros::Publisher _path_pub;
using namespace std;

/////////////////////////////////////////////////////////
//-------------------CallBack!!!!!---------------------//
/////////////////////////////////////////////////////////
boost::mutex va_mutex_;
trajectory_generation::VelocityArray va_; 
void vaCallback(const trajectory_generation::VelocityArrayConstPtr& msg)
{
	boost::mutex::scoped_lock(va_mutex_);
	va_=*msg;
}

boost::mutex v_mutex_;
trajectory_generation::Velocity v_; 
void vCallback(const trajectory_generation::VelocityConstPtr& msg)
{
	boost::mutex::scoped_lock(v_mutex_);
	v_=*msg;
}

boost::mutex odo_mutex_;
nav_msgs::Odometry _odo; 
void tinyCallback(const nav_msgs::OdometryConstPtr& msg)
{
	boost::mutex::scoped_lock(odo_mutex_); 
	_odo=*msg; 
}

nav_msgs::OccupancyGrid grd_;
boost::mutex map_mutex_;
void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg){
	boost::mutex::scoped_lock(map_mutex_);	
	grd_=*msg;
}

/////////////////////////////////////////////////////////
//-------------------Visualize!!!!!--------------------//
/////////////////////////////////////////////////////////
void showPath(nav_msgs::Path path)
{
	std_msgs::ColorRGBA rgba_in;
	rgba_in.r = 0.5;
	rgba_in.g = 0;
	rgba_in.b = 0.7;
	rgba_in.a = 0.8;
	//Visualization* marker_line = new Visualization(rgba_in, ADD,0.1, "CheckPath","/odom");
	Visualization* marker_line = new Visualization(rgba_in, ADD,0.1, "CheckPath","/tiny_odom");
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

bool motionPrediction(float dt, const tf::StampedTransform trans,
						nav_msgs::Path& motion)
{
	trajectory_generation::VelocityArray v_a;
	{
		boost::mutex::scoped_lock(va_mutex_);
		v_a=va_;
	}
	trajectory_generation::Velocity v;
	{
		boost::mutex::scoped_lock(v_mutex_);
		v=v_;
	}
	nav_msgs::Odometry odo; 
	{
		boost::mutex::scoped_lock(odo_mutex_); 
		odo = _odo;
	}
	if(v_a.vel.size() > 0){
		geometry_msgs::PoseStamped x_pre;
		/*------------------------------Initial Setting----------------------------------*/
		x_pre.pose.position.x = trans.getOrigin().x();
		x_pre.pose.position.y = trans.getOrigin().y();
		x_pre.pose.position.z = 0;
		float angle = tf::getYaw(trans.getRotation());
		x_pre.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,angle);
		v_a.vel[v.id].op_linear = odo.twist.twist.linear.x;
		v_a.vel[v.id].op_angular = odo.twist.twist.angular.z;
		/*-------------------------------------------------------------------------------*/
		//for(int i=v.id; i<v_a.vel.size(); i++){
		for(unsigned int i=v.id; i<v_a.vel.size(); i++){
			float pre_angle = tf::getYaw(x_pre.pose.orientation);
			geometry_msgs::PoseStamped x_next;
			x_next.pose.position.x = x_pre.pose.position.x + v_a.vel[i].op_linear*cos(pre_angle)*dt;
			x_next.pose.position.y = x_pre.pose.position.y + v_a.vel[i].op_linear*sin(pre_angle)*dt;
			x_next.pose.position.z = 0;
			float next_angle = pre_angle + v_a.vel[i].op_linear*v_a.vel[i].op_angular*dt;
			x_next.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,next_angle);
			motion.poses.push_back(x_next);
			x_pre = x_next;
		}
		return true;
	}else{
		cout<<"Stanby....."<<endl;
		return false;
	}
}

void collisionCheck()
{
	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe("plan/velocity_array", 100, vaCallback);
	ros::Subscriber sub3 = n.subscribe("/plan/motion_play", 100, vCallback);
	ros::Subscriber sub2 = n.subscribe("/tinypower/odom", 100, tinyCallback);
	//ros::Subscriber sub4 = n.subscribe("map/expanded/map/local", 100, mapCallback);
	ros::Subscriber sub4 = n.subscribe("senior/map", 100, mapCallback);
	ros::Publisher pub1 = n.advertise<std_msgs::Bool>("/plan/change_flag", 1);
	ros::Publisher pub2 = n.advertise<visualization_msgs::Marker>("plan/checkpath_vis", 100);
	{
		boost::mutex::scoped_lock(path_pub_mutex_);	
		_path_pub = pub2;
	}
	
	tf::TransformListener listener;
	std_msgs::Bool flag;
	ros::Rate loop_rate(20);
	while(ros::ok()){
		flag.data = false;
		tf::StampedTransform transform;
		try{
			//listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
			listener.lookupTransform("/tiny_odom", "/base_link", ros::Time(0), transform);
		}catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		nav_msgs::OccupancyGrid gmap;
		{
			boost::mutex::scoped_lock(map_mutex_);
			gmap=grd_;
		}
		nav_msgs::Path motion;
		motionPrediction(dt, transform, motion);
		float cost = pathCheck(motion, gmap);
		if(cost>0){
			flag.data = true;
			cout<<"Change!!"<<endl;
		}
		showPath(motion);
		pub1.publish(flag);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "collision_checker");
	collisionCheck();
	return 0;
}
