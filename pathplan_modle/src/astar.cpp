#include "pathplan_modle/astar.h"



AStar::AStar()
{
    //ROS_INFO("astar modle created!");
}
AStar::~AStar()
{
}
void AStar::get_map_point(nav_msgs::OccupancyGrid & map,
                    geometry_msgs::PoseWithCovarianceStamped & start_point, 
                    geometry_msgs::PoseWithCovarianceStamped & target_point)
{
    ROS_INFO("start_point: x=%f , y=%f",start_point.pose.pose.position.x,start_point.pose.pose.position.y);
    ROS_INFO("target_point: x=%f , y=%f",target_point.pose.pose.position.x,target_point.pose.pose.position.y);
    int i = 0;
    int j = 0;
    vector<float> p_map;
    while(j<map.info.height)
    {
        while(i<map.info.width)
        {
            if(map.data[j*10000+i]!=0) 
            {
                float a = 10000;
                p_map.push_back(10000.0);
            }
            else
            {
                float a=0 ;
                p_map.push_back(a);
            }
            i++;
        }
        plan_map.push_back(p_map);
        p_map.clear();
        i = 0;
        j++;
    }
    expand_all_map();
    start_x = 10*start_point.pose.pose.position.x+5000;
    start_y = 10*start_point.pose.pose.position.y+2500;
    target_x = 10*target_point.pose.pose.position.x+5000;
    target_y = 10*target_point.pose.pose.position.y+2500;
}

void AStar::planning()
{
    Node* start_point = new Node; 
    Node* target_point = new Node;
    start_point->x = start_x;
    start_point->y = start_y;
    start_point->g = 0;
    start_point->father = NULL;
    target_point->x = target_x;
    target_point->y = target_y;
    target_point->father = NULL;
    Node* current_point;
    open_set.push_back(start_point);
     
    bool not_get_target = true;
    int planning_num = 0 ;
    ros::Time start_time, end_time;
    start_time = ros::Time::now();
    ROS_INFO("planning.......");
    while(not_get_target)
    {
        planning_num++;
        //ROS_INFO("planning.......");
        current_point = open_set[0];
        //ROS_INFO("x=%d, y=%d", current_point->x,current_point->y);
        if(current_point->x == target_point->x && current_point->y == target_point->y)
        {
            ROS_INFO("plan ok");
            open_set.clear();
            close_set.clear();
            break;
        }
        extend(current_point);
        close_set.push_back(current_point);
        open_set.erase(open_set.begin());
        MYsort(0,open_set.size()-1,open_set);
    } 
    
    path.header.frame_id = "map";
    Node* find_path_point = current_point;
    while(find_path_point->father!=NULL)
    {
        geometry_msgs::PoseStamped last_point;
        last_point.header.frame_id = "map";
        float x = find_path_point->x;
        float y = find_path_point->y;
        last_point.pose.position.x = (x-5000)/10;
        last_point.pose.position.y = (y-2500)/10;
        //ROS_INFO("x%f y%f",last_point.pose.position.x,last_point.pose.position.y);
        path.poses.push_back(last_point);
        find_path_point = find_path_point->father;
    }
    end_time = ros::Time::now();
    ROS_INFO("path point num = %d",path.poses.size());
    ROS_INFO("planning time = %.2f",end_time.toSec()-start_time.toSec());
    ROS_INFO("planning frequency = %d",planning_num);
    return ;
}


void AStar::extend(Node* current)
{
    //ROS_INFO("start %f",current->f);
    expand_map[current->y][current->x] = 10000;
    extend_check(current->x-1,current->y,current);
    extend_check(current->x+1,current->y,current);
    extend_check(current->x,current->y-1,current);
    extend_check(current->x,current->y+1,current);
    extend_check(current->x-1,current->y-1,current);
    extend_check(current->x+1,current->y+1,current);
    extend_check(current->x-1,current->y+1,current);
    extend_check(current->x+1,current->y-1,current);
    
    return ;
}

void AStar::extend_check(int x,int y,Node* current)
{
    //ROS_INFO("%d,%d",x,y);
    if(x<0||y<0||x>10000||y>5000)
    {
        return;
    }
    if(expand_map[y][x]<=9999)
    {
        Node* new_point = new Node;
        new_point->x = x;
        new_point->y = y;
        float fn = get_f(new_point,current);
        if(expand_map[y][x]==0 || expand_map[y][x]>fn)
        {
            new_point->f = fn;
            new_point->father = current;
            open_set.push_back(new_point);
            expand_map[y][x] = fn;
        }
        else {delete new_point;}
        

    }
}
float  AStar::get_f(Node* new_point,Node* current)
{
    //new_point->f = abs(new_point->x-start_x)*0.2 + abs(new_point->y-start_y)*0.2 + abs(new_point->x-target_x) + abs(new_point->y-target_y);
    float gn = current->g + sqrt((abs(new_point->x-current->x)*abs(new_point->x-current->x)) + (abs(new_point->y-current->y)*abs(new_point->y-current->y)));
    float hn = sqrt((abs(new_point->x-target_x)*abs(new_point->x-target_x)) + (abs(new_point->y-target_y)*abs(new_point->y-target_y)));
    new_point->g = gn;
    new_point->h = hn;
    //new_point->f = gn + hn;
    return (gn + hn);
}

void AStar::MYsort(int x, int y, vector<Node*> & arr)
{
    if(x>=y) return;
    int left = x;
    int right = y;
    float stander = arr[x]->f;
    while(left<=right)
    {
        while(arr[left]->f<stander){left++;}
        while(arr[right]->f>stander){right--;}
        if(left<=right)
        {
            Node* temp = arr[left];
            arr[left] = arr[right];
            arr[right] = temp;
            left++;
            right--;
        }
    }
    MYsort(x,right,arr);
    MYsort(left,y,arr);
    return;
}


void AStar::expand_all_map()
{
    int i = 0;
    int j = 0;
    expand_map = plan_map;
    int expand_num = 8;
    while(i<plan_map.size())
    {
        while(j<plan_map[0].size())
        {
            //ROS_INFO("%f",plan_map[i][j]);
            //ROS_INFO("%d",plan_map[i][j]);
            if(plan_map[i][j] != 0)
            {
                //ROS_INFO("yes");
                int height_low = ((i-expand_num)<0)? 0 : (i-expand_num);
                int height_up = ((i+expand_num)>5000)? 5000 : (i+expand_num);
                int width_low = ((j-expand_num)<0)? 0 : (j-expand_num);
                int width_up = ((j+expand_num)>10000)? 10000 : (j+expand_num);
                for(int m = height_low;m<height_up;m++)
                {
                    for(int n = width_low;n<width_up;n++)
                    {
                        expand_map[m][n] = 10000;
                    }
                }
            }
            j++;
        }
        i++;
        j = 0;
    }
}
