#include <ros/ros.h>
#include <pathplan_modle/pathplansrv.h>
#include "pathplan_modle/astar.h"

bool numcallback(pathplan_modle::pathplansrv::Request & request,
                pathplan_modle::pathplansrv::Response & response)
{
    ros::Time start_time = ros::Time::now();
    AStar astar;
    astar.get_map_point(request.map,request.start_point,request.target_point);
    astar.planning();
    response.path = astar.path;
    ros::Time end_time = ros::Time::now();

    ROS_INFO("all time = %.2f",end_time.toSec()-start_time.toSec());
    return true;
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "astar_server");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("pathinfo",numcallback);
    ros::spin();
    return 0;
}
