#include "ros/ros.h"
#include <trajectory_generation/TrajectoryGeneration.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <sstream>
#include <string>
#include <list>
#include <map>
#include <time.h>

//---------------From library---------------//
#include "trajectory_generation/Visualize_lib.h"
#include "trajectory_generation/trajectory_generation.h"
#include "trajectory_generation/boundary_state.h"
#include "trajectory_generation/LocalPlanning_lib.h"

using namespace std;

//-----------Pathの構造体-----------//
//パス一本一本に対して情報を付加します//
typedef struct {
	float cost;		//優先順位をつけるためのコスト
	nav_msgs::Path path;
	trajectory_generation::VelocityArray v_a;
	u_int id;
	geometry_msgs::PoseStamped goal;
} PathState;

const float Vmin = 0.1;
const float Vmax = 0.5;
const float Lmin = 1.0;
//const float Lmax = 3.0;
const float Lmax = 6.0;
const float MaxAngle = 1.2;


TrajectoryGeneration trajectory(40.0, -40.0,		//curvature
								4.0, -4.0,			//rate of curvature
								0.4, -0.3,				//acceleration
								0.00,					//command delay
								10.1681, -0.0049, 0.9,	//Speed Control logic
								2.0,					//Max curvature for speed
								1.0);					//Safety factor



//ShapeParameter p_ss = {1000, 7, 3, 5.0, -0.8, 0.8, -0.5, 0.5};	//{固定, 目的地数, 目的地あたりの本数, pathの長さ(setting at trajectoryLengthControl), 角度, 角度, 目的地での角度の開き}
ShapeParameter p_ss = {1000, 7, 5, 5.0, -1.2, 1.2, -0.9, 0.9};	//{固定, 目的地数, 目的地あたりの本数, pathの長さ(setting at trajectoryLengthControl), 角度, 角度, 目的地での角度の開き}
//	{n_s, n_p, n_h, d, alph_min, alph_max, psi_min, psi_max}

//////////////////////////////////////////////////
//-------mutexロック スコープの範囲だけ有効-------//
//////////////////////////////////////////////////
ros::Publisher _path_pub;
boost::mutex path_pub_mutex_;
ros::Publisher _pose_pub;
boost::mutex pose_pub_mutex_;
ros::Publisher _array_pub;
boost::mutex array_pub_mutex_;
ros::Publisher _num_pub;
boost::mutex num_pub_mutex_;

/////////////////////////////////////////////////////////
//-------------------CallBack!!!!!---------------------//
/////////////////////////////////////////////////////////
nav_msgs::OccupancyGrid grd_;
boost::mutex map_mutex_;
void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg){
	boost::mutex::scoped_lock(map_mutex_);	
	grd_=*msg;
}

void pathLocalGlobal(nav_msgs::Path& path, geometry_msgs::PoseStamped loc, float vf)
{
	int length=path.poses.size();
	nav_msgs::Odometry zero;
	float angle = tf::getYaw(loc.pose.orientation) - 0.0;	// pathの出る方向
	if(vf<0){
		angle -= 3.141592;
	}
	for(int i=0;i<length;i++){
		float tmp_x = path.poses[i].pose.position.x - zero.pose.pose.position.x;
		float tmp_y = path.poses[i].pose.position.y - zero.pose.pose.position.y;
		float conv_x = cos(angle)*tmp_x - sin(angle)*tmp_y;
		float conv_y = sin(angle)*tmp_x + cos(angle)*tmp_y;
		path.poses[i].pose.position.x = conv_x + loc.pose.position.x;
		path.poses[i].pose.position.y = conv_y + loc.pose.position.y;
	}
}

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

string IntToString(int number)
{
	stringstream ss;
	ss << number;
	return ss.str();
}

void showPathArray(list<PathState> path_array, int num)
{
	std_msgs::ColorRGBA rgba_in;
	rgba_in.r = 0.0;
	rgba_in.g = 0.8;
	rgba_in.b = 0.9;
	rgba_in.a = 0.8;
	//Visualization* marker_line = new Visualization(rgba_in, ADD,0.02, "PathSet","/odom");
	Visualization* marker_line = new Visualization(rgba_in, ADD,0.02, "PathSet","/tiny_odom");
	//Visualization* marker_txt = new Visualization(rgba_in, ADD,0.02, "txtSet","/odom");
	Visualization* marker_txt = new Visualization(rgba_in, ADD,0.02, "txtSet","/tiny_odom");
	ros::Publisher pub1,pub2;
	{
		boost::mutex::scoped_lock(array_pub_mutex_);	
		pub1 = _array_pub;
	}
	{
		boost::mutex::scoped_lock(num_pub_mutex_);	
		pub2 = _num_pub;
	}
	int i = 0;
	//------------------line array---------------------------//
	visualization_msgs::MarkerArray lines, txts;
	list<PathState>::iterator it = path_array.begin();
	while( it != path_array.end() ){
		visualization_msgs::Marker line1, txt1;
		nav_msgs::Path path;
		//if((*it).cost<100)	
		path = (*it).path;
		int id = (*it).id;
		marker_line->convertPath2MarkerLine(path, line1, i);
		marker_txt->txtMarker(IntToString(id), (*it).goal, txt1, i);
		lines.markers.push_back(line1);
		txts.markers.push_back(txt1);
		i++;
		++it;
	}//un-show useless path
	for(; i<num; i++){
		visualization_msgs::Marker line2;
		nav_msgs::Path path_zero;
		marker_line->convertPath2MarkerLine(path_zero, line2, i);
		//lines.markers.push_back(line2);
	}
	
	//------------------txt array---------------------------//
	
	
	pub1.publish(lines);
	
	pub2.publish(txts);
	delete marker_line;
}

void showPoses(geometry_msgs::PoseArray poses)
{
	ros::Publisher pub;
	{
		boost::mutex::scoped_lock(poses_pub_mutex_);
		pub = _pose_pub;
	}
	//poses.header.frame_id = "/odom";
	//poses.header.frame_id = "/tiny_odom";
	poses.header.frame_id = "/base_link";
	poses.header.stamp=ros::Time::now();
	
	pub.publish(poses);
}

///////////////////////////////////////////////////////
/////---------Trajectory Length Control--------////////
///////////////////////////////////////////////////////
float trajectoryLengthControl(float vt)
{
	if(vt < Vmin){
		vt = Vmin;
	}
	if(vt > Vmax){
		vt = Vmax;
	}
	float L = (Lmax - Lmin) * (vt - Vmin)/(Vmax - Vmin) + Lmin;

	return L;
	//return 3.5;
}

float trajectoryLengthControl2(float theta)
{
	float L = -(Lmax-Lmin)*0.637*fabs(theta) + Lmax;	// 0.637 : 2/PI

	//return L;
	//return 3.5;
	return 1.5;
}


///////////////////////////////////////////////////////
/////--------------Target maker----------------////////
///////////////////////////////////////////////////////
void targetMaker(trajectory_generation::TrajectoryGeneration::Request req, 
					geometry_msgs::PoseStamped& x_f)
{
	//p_ss.d = trajectoryLengthControl(req.params.vt);
	float theta = req.goal.pose.position.x;

	printf("\n\ntheta: %f\n\n", theta);

	p_ss.d = trajectoryLengthControl2(theta);
	x_f.pose.position.x = p_ss.d * cos(theta);
	x_f.pose.position.y = p_ss.d * sin(theta);
	x_f.pose.position.z = 0;
	x_f.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,theta);

	// 指向性を持たせる : joyの方向の周囲のみに終端座標群を置く
	p_ss.alph_min = theta - 0.35;	// 0.52 : 30[deg], 0.35 : 20[deg]
	p_ss.alph_max = theta + 0.35;	// 0.52 : 30[deg]
	//if(p_ss.alph_min < -MaxAngle){
	//	p_ss.alph_min = -MaxAngle;
	//	p_ss.alph_max = -MaxAngle+0.7;
	//}
	//if(p_ss.alph_max > MaxAngle){
	//	p_ss.alph_max = MaxAngle;
	//	p_ss.alph_min = MaxAngle-0.7;
	//}
}

///////////////////////////////////////////////////////
/////----------BoundaryStates function---------////////
///////////////////////////////////////////////////////
float setBoundaryStates(geometry_msgs::PoseStamped x_f,
						geometry_msgs::PoseArray& boundary_state)
{
	float return_n;
	geometry_msgs::PoseStamped x_i;
	x_i.pose.position.x=0;
	x_i.pose.position.y=0;
	x_i.pose.position.z=0;
	x_i.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0.0);	// 終端座標群の基準方向
	return_n = (p_ss.n_p * p_ss.n_h);
	//generateGloballyGuidedBoundaryStates(p_ss,x_i, req.goal, boundary_state);
	generateGloballyGuidedBoundaryStates(p_ss,x_i, x_f, boundary_state);
	//generateUniformBoundaryStates(p_ss,x_i,boundary_state);
	showPoses(boundary_state);	// 終端座標群のベクトルを表示
	return return_n;
}

///////////////////////////////////////////////////////
/////-----------Evaluation function------------////////
///////////////////////////////////////////////////////
float smoothness(PathState path)
{
	float smooth_cost=0;
	for(size_t i=0; i<path.v_a.vel.size(); i++){
		float dk = path.v_a.vel[i].op_angular-path.v_a.vel[i+1].op_angular;
		//smooth_cost += (path.v_a.vel[i].op_angular + dk*dk);
		smooth_cost += dk*dk;
	}
	return smooth_cost;
}

float directionCost(geometry_msgs::PoseStamped goal, float param)
{
	float cost;
	cost = param * goal.pose.position.y;
	if( !(fabs(param)>0) ){
		cost = fabs(goal.pose.position.y);
	}
	
	return cost;
}

float distanceToObstacle(geometry_msgs::PoseArray array, geometry_msgs::PoseStamped goal)
{
	float min_dist = INFINITY;
	for(size_t i=0; i<array.poses.size(); i++){
		float dx = goal.pose.position.x - array.poses[i].position.x;
		float dy = goal.pose.position.y - array.poses[i].position.y;
		float cost = sqrt(dx*dx + dy*dy);
		if(min_dist > cost){
			min_dist = cost; 
		}
	}
	cout<<"min_dist="<<min_dist<<endl;
	return min_dist;
}
/*
void pickUpTrajectory(list<PathState> path_array, 
						PathState& pick_up_path, 
						geometry_msgs::PoseStamped goal)
{
	list<PathState>::iterator it = path_array.begin();
	float p_count = 0;
	float min_cost = INFINITY;
	
	for(;it != path_array.end(); ++it){
		float d_cost = directionCost((*it).goal, goal.pose.position.x);
		float dyaw = 0-tf::getYaw((*it).goal.pose.orientation);
		float cost =  d_cost + 0.2*fabs(dyaw) + 2.0*smoothness((*it));
		if(min_cost > cost){
			min_cost = cost;
			pick_up_path = (*it); 
		}
		p_count++;
	}
}*/
int pickUpTrajectory(list<PathState> path_array, 
						PathState& pick_up_path, 
						geometry_msgs::PoseStamped goal)
{
	PathState zero_path;
	geometry_msgs::PoseArray hazard_goal_a;
	int p_count = 0;
	float pre_id = 0;
	float min_cost = INFINITY;
	list<PathState>::iterator it = path_array.begin();
	for(;it != path_array.end(); ++it){
		float num = (*it).id;
		if((num - pre_id) > 1){
			cout<<"jump!"<<endl;
			hazard_goal_a.poses.push_back((*it).goal.pose);
		}
		pre_id = num;
	}
	
	it = path_array.begin();
	for(;it != path_array.end(); ++it){
		float dx = goal.pose.position.x - (*it).goal.pose.position.x;
		float dy = goal.pose.position.y - (*it).goal.pose.position.y;
		float dyaw = tf::getYaw(goal.pose.orientation)-tf::getYaw((*it).goal.pose.orientation);
		//float cost = sqrt(dx*dx + dy*dy) + 1.0*fabs(dyaw) + 0.3*smoothness((*it)) - distanceToObstacle(hazard_goal_a, (*it).goal);
		float cost = sqrt(dx*dx + dy*dy) + 1.0*fabs(dyaw) + 0.3*smoothness((*it));
		//float cost = sqrt(dx*dx + dy*dy) + 0.5*smoothness((*it));
		
		//cout<<"dx "<<dx<<"dy "<<dy<<endl;
		if(min_cost > cost){
			min_cost = cost;
			pick_up_path = (*it); 
		}
		p_count++;
	}
	if(path_array.size()<1)	return -1;
	
	return 0;
}


void generatePathArray(trajectory_generation::TrajectoryGeneration::Request req,
						geometry_msgs::PoseArray boundary_state, 
						list<PathState>& path_array)
{
	
	nav_msgs::OccupancyGrid gmap;
	{
		boost::mutex::scoped_lock(map_mutex_);
		gmap=grd_;
	}
	for(u_int i=0; i<boundary_state.poses.size(); i++){
		PathState path_state;
		float x = boundary_state.poses[i].position.x;
		float y = boundary_state.poses[i].position.y;
		float yaw = tf::getYaw(boundary_state.poses[i].orientation);
		Eigen::Vector3f goal(x, y, yaw);
		
		float speed_ave = (req.params.v0 + req.params.vt + req.params.vf)/3;
		
		Eigen::Vector3f params;
		trajectory.findOptimizedParam(goal, req.params.k0, speed_ave, params);
		
		CntlParam out;
		CntlParam in(CurvParam(req.params.k0, params(0), params(1), params(2)),
					VelParam(req.params.v0,req.params.a0,req.params.vt,req.params.af,req.params.vf));
		
		trajectory_generation::TrajectoryGeneration::Response path_v;
		float tol = trajectory.planning(path_v, out, goal, in, 0.1, 0.05, 10);
		
		path_state.path = path_v.path;
		pathLocalGlobal(path_state.path, req.start, req.params.vf);
		path_state.cost = pathCheck(path_state.path, gmap);
		path_state.v_a = path_v.v_array;
		path_state.id = i;
		path_state.goal.pose = boundary_state.poses[i];//input goal
		if((tol!=-1) && (path_state.cost==0))	path_array.push_back(path_state);
	}
}


bool server(trajectory_generation::TrajectoryGeneration::Request  &req,
         trajectory_generation::TrajectoryGeneration::Response &res )
{
	////cout<<req.fin<<endl;
	////cout<<req.start<<endl;
	////cout<<req.goal<<endl;
	////cout<<req.params<<endl;
	
	clock_t start,end;
	start = clock();
	
	geometry_msgs::PoseStamped x_f;
	targetMaker(req, x_f);
	
	geometry_msgs::PoseArray boundary_state;
	float state_n = setBoundaryStates(x_f, boundary_state);
	
	list<PathState> path_array;
	generatePathArray(req, boundary_state, path_array);
	
	PathState pick_up_path;
	res.tolerance = pickUpTrajectory(path_array, pick_up_path, x_f);
	
	res.path.poses.clear();
	res.v_array.vel.clear();
	res.path = pick_up_path.path;
	res.v_array = pick_up_path.v_a;
	//res.tolerance=tol;
	//res.path.header.frame_id="/odom";
	res.path.header.frame_id="/tiny_odom";
	res.path.header.stamp=req.header.stamp;
	
	showPathArray(path_array, state_n);
	//showPath(res.path);
	
	end = clock();
	//ROS_INFO("Responce Time: %f", (double)(end-start)/CLOCKS_PER_SEC);
	return true;
}


int main(int argc, char **argv)
{
	trajectory.fileInput("/home/amsl/AMSL_ros_pkg/teledrive/look_up_table/infant/v01.bin",0.1);
	trajectory.fileInput("/home/amsl/AMSL_ros_pkg/teledrive/look_up_table/infant/v02.bin",0.2);
	trajectory.fileInput("/home/amsl/AMSL_ros_pkg/teledrive/look_up_table/infant/v03.bin",0.3);
	trajectory.fileInput("/home/amsl/AMSL_ros_pkg/teledrive/look_up_table/infant/v04.bin",0.4);
	trajectory.fileInput("/home/amsl/AMSL_ros_pkg/teledrive/look_up_table/infant/v05.bin",0.5);
	trajectory.fileInput("/home/amsl/AMSL_ros_pkg/teledrive/look_up_table/infant/v06.bin",0.6);
	trajectory.fileInput("/home/amsl/AMSL_ros_pkg/teledrive/look_up_table/infant/v07.bin",0.7);
	trajectory.fileInput("/home/amsl/AMSL_ros_pkg/teledrive/look_up_table/infant/v08.bin",0.8);
	trajectory.fileInput("/home/amsl/AMSL_ros_pkg/teledrive/look_up_table/infant/v09.bin",0.9);

	//trajectory.fileInput("/home/amsl/AMSL_ros_pkg/senior_car_09_28_success/masanobusan/infant_ver/20130928/teledrive/look_up_table/rwrc15/v01.bin",0.1);
	//trajectory.fileInput("/home/amsl/AMSL_ros_pkg/senior_car_09_28_success/masanobusan/infant_ver/20130928/teledrive/look_up_table/rwrc15/v02.bin",0.2);
	//trajectory.fileInput("/home/amsl/AMSL_ros_pkg/senior_car_09_28_success/masanobusan/infant_ver/20130928/teledrive/look_up_table/rwrc15/v03.bin",0.3);
	//trajectory.fileInput("/home/amsl/AMSL_ros_pkg/senior_car_09_28_success/masanobusan/infant_ver/20130928/teledrive/look_up_table/rwrc15/v04.bin",0.4);
	//trajectory.fileInput("/home/amsl/AMSL_ros_pkg/senior_car_09_28_success/masanobusan/infant_ver/20130928/teledrive/look_up_table/rwrc15/v05.bin",0.5);
	//trajectory.fileInput("/home/amsl/AMSL_ros_pkg/senior_car_09_28_success/masanobusan/infant_ver/20130928/teledrive/look_up_table/rwrc15/v06.bin",0.6);
	//trajectory.fileInput("/home/amsl/AMSL_ros_pkg/senior_car_09_28_success/masanobusan/infant_ver/20130928/teledrive/look_up_table/rwrc15/v07.bin",0.7);
	//trajectory.fileInput("/home/amsl/AMSL_ros_pkg/senior_car_09_28_success/masanobusan/infant_ver/20130928/teledrive/look_up_table/rwrc15/v08.bin",0.8);
	//trajectory.fileInput("/home/amsl/AMSL_ros_pkg/senior_car_09_28_success/masanobusan/infant_ver/20130928/teledrive/look_up_table/rwrc15/v09.bin",0.9);
	
	ROS_INFO("Look Up Table complete!!!");
	ros::init(argc, argv, "generate_server");
	ros::NodeHandle n;
	
	ros::Publisher pub3 = n.advertise<visualization_msgs::MarkerArray>("plan/path_recomend", 100);
	ros::Publisher pub4 = n.advertise<visualization_msgs::MarkerArray>("plan/path_num", 100);
	ros::Publisher pub2 = n.advertise<geometry_msgs::PoseArray>("plan/poses", 100);
	ros::ServiceServer service = n.advertiseService("/plan/local_path", server);
	//ros::Subscriber sub1 = n.subscribe("map/expanded/map/local", 100, mapCallback);
	ros::Subscriber sub1 = n.subscribe("senior/map", 100, mapCallback);
	
	{
		boost::mutex::scoped_lock(pose_pub_mutex_);	
		_pose_pub = pub2;
	}
	{
		boost::mutex::scoped_lock(array_pub_mutex_);	
		_array_pub = pub3;
	}
	{
		boost::mutex::scoped_lock(num_pub_mutex_);	
		_num_pub = pub4;
	}
	
	ROS_INFO("generate_server start.");
	
	ros::spin();

	return 0;
}
