
#include "ros/ros.h"
#include <trajectory_generation/TableLookUP.h>
#include <trajectory_generation/TrajectoryGeneration.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include "trajectory_generation//trajectory_generation.h"
#include "math.h"
#include <fstream>
#include <list>
using namespace std;

/*
typedef struct {
	geometry_msgs::PoseStamped path_end;
	CntlParam output;
} GoalPoint;
*/
list<GoalPoint> goal_p;

geometry_msgs::PoseStamped makePath(CntlParam& out, float init_k0, float init_k1, float init_kf, float r){
	
	////TrajectoryGeneration trajectory(1.5, -1.5,		//curvature
	////								0.9, -0.9,			//rate of curvature
	////								0.4, -0.4,				//acceleration
	////								0.00,					//command delay
	////								10.1681, -0.0049, 0.9,	//Speed Control logic
	////								0.9,					//Max curvature for speed
	////								1.0);					//Safety factor
		
	TrajectoryGeneration trajectory(5.5, -5.5,		//curvature
									5.9, -5.9,			//rate of curvature
									5.4, -5.4,				//acceleration
									0.00,					//command delay
									10.1681, -1.0049, 0.9,	//Speed Control logic
									5.9,					//Max curvature for speed
									3.0);					//Safety factor
		



	//CntlParam in(CurvParam(init_k0,init_k1,init_kf,r), VelParam(0.1,0.4,0.1,-0.4,0.1));
	CntlParam in(CurvParam(init_k0,init_k1,init_kf,r), VelParam(0.5,2.4,0.5,-2.4,0.5));
	out = in;
	geometry_msgs::PoseStamped end_p = trajectory.saitekikaSinai(out, 0.1);
	//delete lattice;
	return end_p;
}

void fileOutput()
{
	//ofstream ofs( "/home/amsl/AMSL_ros_pkg/kaihatu/trajectory_generation/look_up_table/infant/v01.bin",ios::binary);
	ofstream ofs( "/home/amsl/AMSL_ros_pkg/senior_car_09_28_success/masanobusan/infant_ver/20130928/teledrive/look_up_table/rwrc15/v05.bin",ios::binary);
	//ofstream ofs( "/opt/ros/groovy/stacks/my_ros/trajectory_generation/look-up-table.bin",ios::binary);
	list<GoalPoint>::iterator it = goal_p.begin();
	while( it != goal_p.end() ){
		float yaw = (*it).path_end.pose.orientation.w;
		float x = (*it).path_end.pose.position.x;
		float y = (*it).path_end.pose.position.y;
		float k0 = (*it).output.curv.k0;
		float k1 = (*it).output.curv.k1;
		float kf = (*it).output.curv.kf;
		float sf = (*it).output.curv.sf;
		float start_num = 1234567;
		//ofs << x << "," << y << "," <<  yaw << "," << k0 << "," << k1 << ","<< kf << ","<< sf <<endl;
		ofs.write(( char * ) &start_num,sizeof( float ) );
		ofs.write(( char * ) &x,sizeof( float ) );
		ofs.write(( char * ) &y,sizeof( float ) );
		ofs.write(( char * ) &yaw,sizeof( float ) );
		ofs.write(( char * ) &k0,sizeof( float ) );
		ofs.write(( char * ) &k1,sizeof( float ) );
		ofs.write(( char * ) &kf,sizeof( float ) );
		ofs.write(( char * ) &sf,sizeof( float ) );
		++it;
	}
}

list<GoalPoint> mabiku(list<GoalPoint> goals, float multiple)
{
	GoalPoint end;
	list<GoalPoint> goal_filted;
	list<GoalPoint>::iterator it = goals.begin();
	while( it != goals.end() ){
		end = (*it);
		int x_int = multiple * (*it).path_end.pose.position.x;
		int y_int = multiple * (*it).path_end.pose.position.y;
		int yaw_int = multiple * (*it).path_end.pose.orientation.w;
		
		end.path_end.pose.position.x = x_int;
		end.path_end.pose.position.y = y_int;
		end.path_end.pose.orientation.w = yaw_int;
		
		bool overlap = false;
		list<GoalPoint>::iterator it2 = goal_filted.begin();
		while( !overlap && (it2 != goal_filted.end()) ){
			if((roundf((*it).path_end.pose.position.x*multiple)==roundf((*it2).path_end.pose.position.x*multiple)) 
				&& (roundf((*it).path_end.pose.position.y*multiple)==roundf((*it2).path_end.pose.position.y*multiple))
				&& (roundf((*it).path_end.pose.orientation.w*multiple)==roundf((*it2).path_end.pose.orientation.w*multiple))){
				overlap = true;
			}	
			++it2;
		}
		if(it2 == goal_filted.end()&& !overlap){
			end.path_end.pose.position.x /= multiple;
			end.path_end.pose.position.y /= multiple;
			end.path_end.pose.orientation.w /= multiple;
			goal_filted.push_back( (*it) );
		}
		++it;
	}
	cout<<"jmabiitaato = "<<goal_filted.size()<<endl;
	return goal_filted;
}

void init(float k_min,float k_max,float k_d,float sf_min,float sf_max,float sf_d){
	GoalPoint p;
	int path_count = 0;
	CurvParam param(0,0,0,0);
	goal_p.clear();
	for(float k0 = k_min; k0 <= k_max; k0 += k_d){
		for(float k1 = k_min; k1 <= k_max; k1 += k_d){
			for(float kf = k_min; kf <= k_max; kf += k_d){
				for(float sf = sf_min; sf <= sf_max; sf += sf_d){
					CntlParam out;
					p.path_end = makePath(out, k0, k1, kf, sf);
					p.output = out;
					goal_p.push_back(p);
					path_count++;
				}
			}
		}
		cout<<"k0="<<k0<<"\t"<< round(100*(k0-k_min)/(k_max-k_min)) <<"%"<<endl;
	}
	//goal_p = mabiku(goal_p,10);
	
	cout<<"table was created !!"<<endl;
	cout<<"Path is "<<path_count<<" patern!!!!!!!!!"<<endl;
}

void showPose(ros::Publisher pub)
{
	geometry_msgs::PoseArray path_ends;
	geometry_msgs::Pose end;
	list<GoalPoint>::iterator it = goal_p.begin();
	path_ends.header.frame_id="/global";
	while( it != goal_p.end() ){
		float yaw = (*it).path_end.pose.orientation.w;
		geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);
		end.orientation = pose_quat;
		end.position.x = (*it).path_end.pose.position.x;
		end.position.y = (*it).path_end.pose.position.y;
		path_ends.poses.push_back(end);
		++it;
	}
	pub.publish(path_ends);
}

int main(int argc, char **argv)
{
	init(-1.0, 10.0, 0.10, 1, 8, 1.0);
	ros::init(argc, argv, "look_up_table_creator");
	ros::NodeHandle n;
	ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseArray>("plan/table_pose", 100);
	fileOutput();
	ros::Rate loop_rate(10);	
	while(ros::ok()){
		showPose(pose_pub);
		loop_rate.sleep();
	}

	return 0;
}

