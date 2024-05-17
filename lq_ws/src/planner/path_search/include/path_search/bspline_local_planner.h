#ifndef _BSPLINE_LOCAL_PLANNER
#define _BSPLINE_LOCAL_PLANNER

#include <ros/ros.h>
#include "path_search/bspine.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/Odometry.h"
#include <eigen3/Eigen/Eigen>
#include <unordered_map>
#include <functional>

class Bspline_planner
{
private:

    nav_msgs::Path mGlobalPath; bool mGlobalPathFlag=0;
    nav_msgs::Path mLobalPath;
    nav_msgs::OccupancyGrid mLocalMap; bool mLocalMapFlag=0;
    nav_msgs::Odometry mOdom; bool mOdomFlag=0;

    // std::unordered_map<std::string,bool> mCheckMap;
    double checkgridlength = 10.0;
    int checkgridnum = 1;
    bool mleftflag = true;

    int mStartindex = 0;
    int mLocalPathLength = 25;
    double resulotion = 1.0;
    nav_msgs::Path spline_path;

    bool init();
    void NearedIndex(const nav_msgs::Path &path, const nav_msgs::Odometry &odom, int &index);
    void GetpointOfGlobalpath(std::vector<int> Waypoints, std::vector<std::vector<double>> &ConWaypoints);
    void GetpointOfGlobalpath(const nav_msgs::Path &globalpath, int start, int pianyi, std::vector<std::vector<double>> &ConWaypoints);
    bool CollitionAvoid(std::vector<std::vector<double>> &ConWaypoints, const nav_msgs::OccupancyGrid &map);
    bool renewpoint(std::vector<std::vector<double>> &ConWaypoints, int pointindex);
    void expandSearch(std::vector<std::vector<double>> &points);
    bool IsObstable(std::vector<double> point);
    nav_msgs::Path bspline_path(const std::vector<std::vector<double>> &points);
public:
    Bspline_planner();
    ~Bspline_planner();
    bool SplinePathSearch();
    bool SplinePathSearch(const nav_msgs::Path &globalpath, const nav_msgs::OccupancyGrid &localmap,const nav_msgs::Odometry odom);

    void GlobalPathCB(const nav_msgs::Path &msg);
    void LocalMapCB(const nav_msgs::OccupancyGrid &msg);
    void OdometryCB(const nav_msgs::Odometry &msg);

    nav_msgs::Path GetSplinePath();
};

#endif