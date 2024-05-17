#ifndef _PATH_SEARCH_H_
#define _PATH_SEARCH_H_

#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
// #include <Eigen/Eigen> //18.04
#include <eigen3/Eigen/Eigen>
#include <queue>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include "path_search/bspine.h"
#include "path_search/dubins.h"

struct GridNode;
typedef GridNode *GridNodePtr;

static geometry_msgs::Quaternion default_quatation;

struct waypoint{
    double x;
    double y;
    double angle;
};

struct GridNode
{
    enum enum_state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3,
        OBSTACLE = 4
	};

	enum enum_state state
	{
		UNDEFINED
	};
	Eigen::Vector2i index;
    Eigen::Vector2d vel = Eigen::Vector2d::Zero();
	double angle = 0;

	double gScore{inf}, hScore{inf};
	GridNodePtr cameFrom{NULL};
};

class NodeComparator
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->hScore > node2->hScore;
	}
};

class path_search
{
private:
    ros::Rate rate = 1;

    // tybspline mbspline;
    geometry_msgs::PoseWithCovarianceStamped initpose;
    geometry_msgs::PoseStamped Startpose,Endpose; bool start_flag = false, end_flag = false;
    Eigen::Vector2i Start_index, current_index, End_index;
    double mStartAngle, mEndAngle;  //起点朝向和终点朝向。 
    double map_leng_x; double map_leng_y;
    int map_index_x; int map_index_y;
    int index_sum;
    double mMinRadius = 20.0; //最小转弯半径。
    bool occumap_flag = false;
    GridNodePtr **NodeMap = nullptr; bool nodemap_flag = false;
    bool costmap_flag = false;

    double resolution = 0.259;
    double weight = 4.0;

    GridNodePtr current_nodeptr;
    // GridNodePtr end_nodeptr;
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet;

	std::vector<GridNode> NodePath;
    nav_msgs::Path PubPath;

    std::vector<waypoint> newreplanpoint;
    nav_msgs::Path thefirstpath;
    int firstpathindex;

    bool init_Curve_Nodemap();
    void relieveNodemap(GridNodePtr **NodeMap);

    double GetHeu(Eigen::Vector2i index);
    
    bool CheckCurrentPose(GridNodePtr nodeptr);
    bool OutOfrange(const Eigen::Vector2i index);

    void Set_obstacle(GridNodePtr nodeptr);
    bool run();

    bool Oneshot(Eigen::Vector2d pt1,Eigen::Vector2d pt2, nav_msgs::Path &partpath);
    bool newreplan(const std::vector<waypoint> &points, nav_msgs::Path &path);
    bool astarOneshot(const std::vector<waypoint> &points, nav_msgs::Path &path);

    bool pointsinit(const waypoint &start,const waypoint &end, const nav_msgs::OccupancyGrid &map);
    bool Getpath();
    nav_msgs::Path bspline_path(const nav_msgs::Path &rawpath);

    bool path_flag = false;

    inline bool Index2Coord(const Eigen::Vector2i index, geometry_msgs::PoseStamped &coord) const;
    inline bool Coord2Index(Eigen::Vector2i &index, const geometry_msgs::PoseStamped coord) const;
    inline double Cul_distance(const Eigen::Vector2i point1,const Eigen::Vector2i point2) const;
    inline bool Is_obstacle(const GridNodePtr nodeptr)const;
    inline Eigen::Vector3d Quataition2Euler(const geometry_msgs::Quaternion ge_quat)const;

public:
    nav_msgs::OccupancyGrid Cost_Map; 
    nav_msgs::Path PubPathCircle11;
    nav_msgs::Path PubPathCircle12;
    nav_msgs::Path PubPathCircle21;
    nav_msgs::Path PubPathCircle22;
    visualization_msgs::Marker Make_point1;

    path_search();
    ~path_search();
    bool Getpathmain(nav_msgs::Path &path);
    //该库函数的主要使用入口，path用于接收路径。
    bool Getpathmutiple(const std::vector<waypoint> &points, nav_msgs::Path &path, const geometry_msgs::Quaternion &quaAngle);

    bool replan(geometry_msgs::PoseStamped odompose, nav_msgs::Path &path, const nav_msgs::OccupancyGrid &map);

    void OccumapCB(const nav_msgs::OccupancyGrid &msg);
    void StartposeCB(const geometry_msgs::PoseWithCovarianceStamped &msg);
    void EndposeCB(const geometry_msgs::PoseStamped &msg);

};

inline bool path_search::Index2Coord(const Eigen::Vector2i index, geometry_msgs::PoseStamped &coord) const
{
    double x = (index[0]) * resolution + Cost_Map.info.origin.position.x;
    double y = (index[1]) * resolution + Cost_Map.info.origin.position.y;

    coord.pose.position.x = x;
    coord.pose.position.y = y;
    coord.pose.position.z = 0;

    return true;
};

inline bool path_search::Coord2Index(Eigen::Vector2i &index, const geometry_msgs::PoseStamped coord) const
{
    int x = int((coord.pose.position.x - Cost_Map.info.origin.position.x) / resolution);
    int y = int((coord.pose.position.y - Cost_Map.info.origin.position.y) / resolution);

    index[0] = x;
    index[1] = y;

    return true;
};

inline double path_search::Cul_distance(const Eigen::Vector2i point1,const Eigen::Vector2i point2) const{  
    double dx =  point1[0] - point2[0];
    double dy =  point1[1] - point2[1];
    double num =  sqrt(dx*dx + dy*dy);
    double ansnum = resolution * num;
    return ansnum;
}

inline bool path_search::Is_obstacle(const GridNodePtr nodeptr) const{
    int inX = nodeptr->index[0];
    int inY = nodeptr->index[1];

    for(int i = -2; i <= 2; ++i)
        for(int j = -2; j <= 2; ++j)
            {
                int indexxx = inX + i;
                int indeyyy = inY + j;
                if(indexxx < 0 || indexxx >= 800 || indeyyy >=800 || indeyyy < 0)
                    continue;
                
                if(NodeMap[indexxx][indeyyy]->state == GridNode::OBSTACLE){
                    // std::cout << "indexxx " <<indexxx << std::endl;
                    // std::cout << "indexxx " <<indeyyy << std::endl;
                    return true;
                }
            }

    return false;
    // return nodeptr->state == GridNode::OBSTACLE;
}

inline Eigen::Vector3d path_search::Quataition2Euler(const geometry_msgs::Quaternion ge_quat) const{
    Eigen::Vector3d euler;

    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(ge_quat, tf_quat);

    tf::Matrix3x3(tf_quat).getRPY(euler[0], euler[1], euler[2]);
    return euler;
}

#endif