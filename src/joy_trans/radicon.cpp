//--------------------------------- < AMSL > ----------------------------------
/* 
 * file         :       Radicon.cpp 
 * 
 * 
 * Environment  :       g++ 
 * Latest Update:       2010/03/01
 * 
 * Designer(s)  :       3110 (AMSL) 
 * Author(s)    :       3110 (AMSL)    
 * 
 * CopyRight    :       2010, Autonomous Mobile Systems Laboratory, Meiji Univ. 
 * 
 * Revision     :       2010/03/01      new
 * 			
 */ 
//-----------------------------------------------------------------------------

#include <iostream>
#include <ros/ros.h>
//#include <turtlesim/Velocity.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>


class TeleopRadicon
{
public:
  TeleopRadicon();

private:
	void visualizeInit(visualization_msgs::Marker& arrow);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void piToPi(float& angle);
	ros::NodeHandle nh_;

	int linear_, angular_;
	double l_scale_, a_scale_;
	ros::Publisher direction_pub_;
	ros::Publisher angle_pub_;
	ros::Publisher arrow_pub_;
	ros::Subscriber joy_sub_;
	
	tf::TransformListener listener;

};

TeleopRadicon::TeleopRadicon():
  linear_(1),
  angular_(3),
l_scale_(0.4),
//  l_scale_(0.3),
  a_scale_(0.4)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  //vel_pub_ = nh_.advertise<turtlesim::Velocity>("turtle1/command_velocity", 1);
  direction_pub_ = nh_.advertise<geometry_msgs::Twist>("plan/direction", 1);
  angle_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("plan/angle", 1);
  arrow_pub_ = nh_.advertise<visualization_msgs::Marker>("plan/vector", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopRadicon::joyCallback, this);
}

void TeleopRadicon::visualizeInit(visualization_msgs::Marker& arrow)
{
	arrow.header.frame_id="/odom";
	arrow.header.stamp=ros::Time::now();
	arrow.ns="arrow";
	arrow.action = visualization_msgs::Marker::ADD;
	arrow.id=0; // set each ID number of arrow
	arrow.type=visualization_msgs::Marker::ARROW;
		
	arrow.pose.position.x=0;
	arrow.pose.position.y=0;
	arrow.pose.position.z=0;
	arrow.pose.orientation.x = 0.0;
	arrow.pose.orientation.y = 0.0;
	arrow.pose.orientation.z = 0.0;
	arrow.pose.orientation.w = 0.0;

	arrow.scale.x=0.8;
	arrow.scale.y=0.8;
	arrow.scale.z=0.0;

	arrow.color.g=0.8;
	arrow.color.r=0.1;
	arrow.color.b=0.0;
	arrow.color.a=1.0;
}

void TeleopRadicon::piToPi(float& angle)
{
	if(angle > M_PI)		angle -= 2*M_PI;
	else if(angle < -M_PI)	angle += 2*M_PI;
}

void TeleopRadicon::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist direction;
	geometry_msgs::PoseStamped angle_p;
	visualization_msgs::Marker arrow;
	visualizeInit(arrow);
	
	float angle = 0;
	direction.linear.x = l_scale_ * joy->axes[linear_];
	//direction.angular.z = joy->axes[angular_];
	
	if(joy->axes[3] >= 0.99){
		angle = -M_PI_4*joy->axes[4] + M_PI_2;
	}else if(joy->axes[4] >= 0.99){
		angle = M_PI_4*joy->axes[3];
	}else if(joy->axes[3] <= -0.99){
		angle = M_PI_4*joy->axes[4] - M_PI_2;
	}else if(joy->axes[4] <= -0.99){
		angle = -M_PI_4*joy->axes[3] - M_PI;
	}
	
	piToPi(angle);
	direction.angular.z = angle;
	tf::StampedTransform transform;
	try{
		listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	angle_p.pose.position.x = transform.getOrigin().x();
	angle_p.pose.position.y = transform.getOrigin().y();
	angle_p.pose.position.z = 0;
	float yaw = tf::getYaw(transform.getRotation());
	angle_p.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,angle+yaw);
	angle_p.header.frame_id = "/odom";
	angle_p.header.stamp=ros::Time::now();
	
	arrow.pose = angle_p.pose;
	arrow.scale.z = 2*fabs(direction.linear.x);
	
	if( joy->buttons[2] == 1 ){
		std::cout<<"s!!!"<<std::endl;
		
	}if( joy->buttons[0] == 1 ){
		std::cout<<"l!!!"<<std::endl;
		
	}if( joy->buttons[1] == 1 ){
		std::cout<<"o!!!"<<std::endl;
		
	}if( joy->buttons[5] == 1 ){
		std::cout<<"w!!!"<<std::endl;
		
	}

	direction_pub_.publish(direction);
	angle_pub_.publish(angle_p);
	arrow_pub_.publish(arrow);
	
	printf("direction.linear.x = \t %2.3f \n",direction.linear.x);
	printf("direction.angular.z = \t %2.3f \n\n",direction.angular.z);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "radicon");
	
	TeleopRadicon radicon;
	
	ros::spin();
	
	
}

