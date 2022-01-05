#ifndef ASTAR_H
#define ASTAR_H


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
#include <math.h>
using namespace std;

typedef struct Node
{
	int x,y;
	float g; 
	float h;
	float f;
	Node* father;

}Node;

class AStar{
    public:
    AStar();
    ~AStar();
	vector< vector<float> > plan_map;
    vector< vector<float> > expand_map;
	int start_x;
    int start_y;
    int target_x;
    int target_y;
    vector<Node*> open_set;
    vector<Node*> close_set;
    nav_msgs::Path path;

    void expand_all_map();

    void get_map_point(nav_msgs::OccupancyGrid & map,geometry_msgs::PoseWithCovarianceStamped & start_point, geometry_msgs::PoseWithCovarianceStamped & target_point);
	void planning();
    void extend(Node* point);
    void extend_check(int x,int y,Node* current);
    void MYsort(int x, int y, vector<Node*> & arr);
    float get_f(Node* new_point,Node* current);
};
	


#endif