#include "ros/ros.h"
#include <trajectory_generation/TrajectoryGeneration.h>
#include <geometry_msgs/PoseArray.h>
#include <fstream>
#include <string>
#include <list>
#include <map>
#include <time.h>

//---------------From library---------------//
#include "library/Visualize_lib.h"
#include "library/trajectory_generation.h"
#include "library/boundary_state.h"


using namespace std;

ros::Publisher _path_pub;
boost::mutex path_pub_mutex_;
ros::Publisher _pose_pub;
boost::mutex pose_pub_mutex_;
ros::Publisher _array_pub;
boost::mutex array_pub_mutex_;

//-----------Pathの構造体-----------//
//パス一本一本に対して情報を付加します//
typedef struct {
	float cost;		//優先順位をつけるためのコスト
	nav_msgs::Path path;
	int id;
} PathState;

TrajectoryGeneration trajectory(0.5, -0.5,		//curvature
								0.3, -0.3,			//rate of curvature
								0.4, -0.4,				//acceleration
								0.08,					//command delay
								10.1681, -0.0049, 0.9,	//Speed Control logic
								0.4,					//Max curvature for speed
								1.0);					//Safety factor


void showPath(nav_msgs::Path path)
{
	std_msgs::ColorRGBA rgba_in;
	rgba_in.r = 1.0;
	rgba_in.g = 0;
	rgba_in.b = 0;
	rgba_in.a = 0.8;
	Visualization* marker_line = new Visualization(rgba_in, ADD,0.1, "OnlyPath","/global");
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

void showPathArray(list<PathState> path_array)
{
	std_msgs::ColorRGBA rgba_in;
	rgba_in.r = 0.0;
	rgba_in.g = 0.8;
	rgba_in.b = 0.9;
	rgba_in.a = 0.8;
	Visualization* marker_line = new Visualization(rgba_in, ADD,0.02, "PathSet","/global");
	ros::Publisher pub;
	{
		boost::mutex::scoped_lock(array_pub_mutex_);	
		pub = _array_pub;
	}
	int i = 0;
	visualization_msgs::MarkerArray lines;
	list<PathState>::iterator it = path_array.begin();
	while( it != path_array.end() ){
		visualization_msgs::Marker line1;
		nav_msgs::Path path;
		//if((*it).cost<100)	
		path = (*it).path;
		marker_line->convertPath2MarkerLine(path, line1, i);
		lines.markers.push_back(line1);
		i++;
		++it;
	}
	pub.publish(lines);
	delete marker_line;
}

void showPoses(geometry_msgs::PoseArray poses)
{
	ros::Publisher pub;
	{
		boost::mutex::scoped_lock(poses_pub_mutex_);	
		pub = _pose_pub;
	}
	poses.header.frame_id = "/global";
	poses.header.stamp=ros::Time::now();
		
	pub.publish(poses);
}

void pickUpTrajectory(list<PathState> path_array, 
						PathState pick_up_path, 
						geometry_msgs::PoseStamped goal)
{
	
}

bool generatePath(trajectory_generation::TrajectoryGeneration::Request  &req,
         trajectory_generation::TrajectoryGeneration::Response &res )
{
	ROS_INFO("Server called");
	geometry_msgs::PoseStamped x_i;
	x_i.pose.position.x=0;
	x_i.pose.position.y=0;
	x_i.pose.position.z=0;
	x_i.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
	geometry_msgs::PoseArray boundary_state;
	ShapeParameter p_ss = {1000, 20, 3, 7.0, -0.9, 0.9, -0.71, 0.71};
	list<PathState> path_array;
	generateUniformBoundaryStates(p_ss,x_i,boundary_state);
	//generateGloballyGuidedBoundaryStates(p_ss,x_i,req.goal,boundary_state);
	clock_t start,end;
	start = clock();

	u_int state_n = boundary_state.poses.size();
	for(u_int i=0; i<state_n; i++){
		try{
			PathState path_state;
			GoalPoint saiteki;
			float x = boundary_state.poses[i].position.x,	y = boundary_state.poses[i].position.y,	yaw = tf::getYaw(boundary_state.poses[i].orientation);
			//ROS_INFO("look up!");
			saiteki = trajectory.findOptimizedParam(x,y,yaw,req.params.k0,req.params.vt);
			//ROS_INFO("make path");
			CntlParam out, in(CurvParam(req.params.k0,saiteki.output.curv.k1,saiteki.output.curv.kf,saiteki.output.curv.sf),
				 VelParam(req.params.v0,req.params.a0,req.params.vt,req.params.af,req.params.vf));
			
			Eigen::Vector3f goal(x, y, yaw);
			trajectory_generation::TrajectoryGeneration::Response path_v;
			float tol=trajectory.planning(path_v,out, goal, in, 0.1, 0.05, 10);
			
			path_state.path = path_v.path;
			
			res.path.poses.clear();
			res.v_array.vel.clear();
			res.path = path_v.path;
			res.v_array = path_v.v_array;
			
			res.params.k0=out.curv.k0;
			res.params.k1=out.curv.k1;
			res.params.kf=out.curv.kf;
			res.params.sf=out.curv.sf;
			res.params.v0=out.velocity.v0;
			res.params.a0=out.velocity.a0;
			res.params.vt=out.velocity.vt;
			res.params.af=out.velocity.af;
			res.params.vf=out.velocity.vf;
			res.tolerance=tol;
			res.path.header.frame_id="/global";
			res.path.header.stamp=req.header.stamp;
			if(tol!=-1)	path_array.push_back(path_state);
			
			
		}catch(...){
			ROS_ERROR("Failed to Trajectory Generation");
			//return false;
		}
	}
	/*
	res.path.poses.clear();
	res.v_array.vel.clear();
	
	trajectory_generation::TrajectoryGeneration::Response pick_up_path;*/
	//pickUpTrajectory(path_array, pick_up_path, req.goal);
	//
	
	//res.path = path_v.path;
	//res.v_array = path_v.v_array;
	/*res.params.k0=out.curv.k0;
	res.params.k1=out.curv.k1;
	res.params.kf=out.curv.kf;
	res.params.sf=out.curv.sf;
	res.params.v0=out.velocity.v0;
	res.params.a0=out.velocity.a0;
	res.params.vt=out.velocity.vt;
	res.params.af=out.velocity.af;
	res.params.vf=out.velocity.vf;
	res.tolerance=tol;
	res.path.header.frame_id="/global";
	res.path.header.stamp=req.header.stamp;*/
	
	end = clock();
	ROS_INFO("Time: %f", (double)(end-start)/CLOCKS_PER_SEC);
	showPathArray(path_array);
	showPath(res.path);
	ROS_INFO("Responce");
	return true;
}


int main(int argc, char **argv)
{
	trajectory.fileInput("/home/amsl/AMSL_ros_pkg/kaihatu/trajectory_generation/look-up-table6.bin",0);
	trajectory.fileInput("/home/amsl/AMSL_ros_pkg/kaihatu/trajectory_generation/look-up-table36.bin",1);
	trajectory.fileInput("/home/amsl/AMSL_ros_pkg/kaihatu/trajectory_generation/look-up-table03.bin",2);
	/*
	fileInput("/opt/ros/groovy/stacks/my_ros/trajectory_generation/look-up-table6.bin",0);
	fileInput("/opt/ros/groovy/stacks/my_ros/trajectory_generation/look-up-table36.bin",1);
	fileInput("/opt/ros/groovy/stacks/my_ros/trajectory_generation/look-up-table03.bin",2);
	*/
	ros::init(argc, argv, "generate_server");
	ros::NodeHandle n;
	
	ros::Publisher pub1 = n.advertise<visualization_msgs::Marker>("plan/localpath_vis", 100);
	ros::Publisher pub3 = n.advertise<visualization_msgs::MarkerArray>("plan/path_recomend", 100);
	ros::Publisher pub2 = n.advertise<geometry_msgs::PoseArray>("plan/poses", 100);
	ros::ServiceServer service = n.advertiseService("/plan/local_path", generatePath);
	{
		boost::mutex::scoped_lock(path_pub_mutex_);	
		_path_pub = pub1;
	}
	{
		boost::mutex::scoped_lock(pose_pub_mutex_);	
		_pose_pub = pub2;
	}
	{
		boost::mutex::scoped_lock(array_pub_mutex_);	
		_array_pub = pub3;
	}
	ROS_INFO("generate_server start.");
	ros::spin();

	return 0;
}
