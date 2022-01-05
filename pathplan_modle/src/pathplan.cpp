#include"pathplan_modle/pathplan.h"
#include <pathplan_modle/pathplansrv.h>

PathPlan::PathPlan()
{
    point_num = 0;
    get_two_points = false;
    ROS_INFO("pathplan modle created!");
}
PathPlan::~PathPlan()
{

}
void PathPlan::init()
{ 
    ROS_INFO("init");
    sub_map = nh.subscribe("/map",1000,&PathPlan::mapcallback,this);
    sub_point = nh.subscribe("initialpose",1000,&PathPlan::pointcallback,this);
    pub_path = nh.advertise<nav_msgs::Path>("planning_path",1);
    client = nh.serviceClient<pathplan_modle::pathplansrv>("pathinfo");
}
void PathPlan::run()
{
    ros::Rate loop_rate(10);
    bool generate_ok = false;
    while(ros::ok())
	{
        
        if(point_num == 2&&get_two_points)
        {
            AStar astar;
            ROS_INFO("path planning start");
            ROS_INFO("start_point: x=%f , y=%f",start_point.pose.pose.position.x,start_point.pose.pose.position.y);
            ROS_INFO("target_point: x=%f , y=%f",target_point.pose.pose.position.x,target_point.pose.pose.position.y);
            get_two_points = false;
            point_num = 0;
            //astar.get_map_point(map,start_point,target_point);
            //astar.planning();
            //path = astar.path;
            pathplan_modle::pathplansrv res;
            res.request.map = map;
            res.request.start_point = start_point;
            res.request.target_point = target_point;
            bool flag = client.call(res);
            path = res.response.path;
            generate_ok = true;
        }
        if(generate_ok)
        {
            pub_path.publish(path);
        }
        
		ros::spinOnce();
		loop_rate.sleep();
	}
}
void PathPlan::mapcallback(const nav_msgs::OccupancyGrid & map_input)
{
    map = map_input;
    ROS_INFO("map size = %d",map.data.size());
    ROS_INFO("resolution = %f , width = %d  , height = %d , x=%f,y=%f,z=%f",
            map.info.resolution,map.info.width,map.info.height,map.info.origin.position.x,map.info.origin.position.y,map.info.origin.position.z);
}
void PathPlan::pointcallback(const geometry_msgs::PoseWithCovarianceStamped & point )
{
    point_num++;
    if(!get_two_points)
    {
        if(point_num == 1)
        {
            ROS_INFO("get start point");
            start_point = point;
        }
        if(point_num == 2)
        {
            ROS_INFO("get target point");
            target_point = point;
            get_two_points = true;
        }
    }
}

int main(int argc, char * argv[])
{
    ros::init(argc,argv,"pathplan");
    PathPlan plan;
    plan.init();
    plan.run();
}
