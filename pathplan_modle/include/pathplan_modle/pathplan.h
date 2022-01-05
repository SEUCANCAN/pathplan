#ifndef PATHPLAN_H
#define PATHPLAN_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <vector>
#include "pathplan_modle/astar.h"

using namespace std;


class PathPlan{
    public:
    PathPlan();
    ~PathPlan();
    ros::NodeHandle nh;
	ros::Subscriber sub_map;
	ros::Subscriber sub_point;
	ros::Publisher pub_path;
	ros::ServiceClient client;

	nav_msgs::Path path;

	int point_num;
	nav_msgs::OccupancyGrid  map;
	geometry_msgs::PoseWithCovarianceStamped start_point;
	geometry_msgs::PoseWithCovarianceStamped target_point;
	bool get_two_points;

    void init();
	void mapcallback(const nav_msgs::OccupancyGrid & map);
	void pointcallback(const geometry_msgs::PoseWithCovarianceStamped & point );
	void run();
};

#endif
