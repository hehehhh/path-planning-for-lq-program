#ifndef _DUBINS_H_
#define _DUBINS_H_

#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <nav_msgs/OccupancyGrid.h>
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>
#include <visualization_msgs/Marker.h>

constexpr double inf = 1 >> 20;

enum linetype //六种曲线的类型。
{
    LSL = 0,
    LSR = 1,
    RSL = 2,
    RSR = 3,
    LRL = 4,
    RLR = 5
};

struct pointnode{
    double x;
    double y; 
    double angle = 0.0;
};

static Eigen::Vector2d default_circle(0, 0);

class tydubins
{
private:
    pointnode start,end;

    std::vector<double> lengths = {inf,inf,inf,inf,inf,inf}; //0-5线段类型。
    double resolution;
    double mMinra = 20.0;//最小转弯半径
    Eigen::Vector2d centre11; //起点左边的园
    Eigen::Vector2d centre12; //起点右边的园
    Eigen::Vector2d centre21; //终点左边的园
    Eigen::Vector2d centre22; //终点右边的院
    //1,2为LSL的两个切点，3，4为LSR的两个切点。依次类推。 6种线段6种线段。
    Eigen::Vector2d Cpoint1,Cpoint2,Cpoint3,Cpoint4,Cpoint5,Cpoint6,
                    Cpoint7,Cpoint8,Cpoint9,Cpoint10,Cpoint11,Cpoint12;

    std::map<double,int> lineListS;
    //第一个存储长度，第二个存储该长度对应的曲线类型。 此map会自动排序，第一个为最短路径

    nav_msgs::Path mDubinsPath;
    nav_msgs::Path mtempPathList[6];

    int map_index_x = 0;
    int map_index_y = 0;
    nav_msgs::OccupancyGrid MAP;
    void GetPointOfContact(const Eigen::Vector2d centre1, const Eigen::Vector2d centre2,Eigen::Vector2d &point1, 
    Eigen::Vector2d &point2, int type, Eigen::Vector2d &Circle = default_circle);

    double Cul_S(const Eigen::Vector2d position1, const Eigen::Vector2d position2, 
    const Eigen::Vector2d centre1, const Eigen::Vector2d centre2, 
    const Eigen::Vector2d point1, const Eigen::Vector2d point2, 
    const int type,std::map<double, int> &lineListS);

    double PureDubinsCul_S(const Eigen::Vector2d position1, const Eigen::Vector2d position2, 
    const Eigen::Vector2d centre1, const Eigen::Vector2d centre2, 
    const Eigen::Vector2d point1, const Eigen::Vector2d point2, 
    const int type,std::map<double, int> &lineListS);

    void GetPointOfContactPub(const Eigen::Vector2d point1,const Eigen::Vector2d point2, const Eigen::Vector2d point3, const Eigen::Vector2d point4,
    const Eigen::Vector2d point5,const Eigen::Vector2d point6, const Eigen::Vector2d point7, const Eigen::Vector2d point8);
    void InsertPubPointEA(const Eigen::Vector2d coord);

    inline Eigen::Vector2d GetCentreCircle(const pointnode node,const bool left) const;
    inline double Cul_triangle_cos(const Eigen::Vector2d point1,const Eigen::Vector2d point2,const Eigen::Vector2d point3) const;
    inline double Cul_distance(const Eigen::Vector2i point1,const Eigen::Vector2i point2) const;
    inline bool Index2Coord(const Eigen::Vector2i index, geometry_msgs::PoseStamped &coord) const;
    inline bool Is_obstacle(const Eigen::Vector2d obscoord) const;

public:
    tydubins();
    tydubins(std::vector<int> start, std::vector<int> end, nav_msgs::OccupancyGrid map);

    ~tydubins();
    
    bool Easyconect(pointnode start, pointnode end, nav_msgs::OccupancyGrid map);
    bool PureDubins(pointnode start, pointnode end);
    nav_msgs::Path GetdubinsPath();

    //以下五行为测试用。
    void Getpathcircle(const Eigen::Vector2d circle11, const Eigen::Vector2d circle12, const Eigen::Vector2d circle21, const Eigen::Vector2d circle22);//检测园用，不能用
    visualization_msgs::Marker Make_point1;
    nav_msgs::Path PubPathCircle11;
    nav_msgs::Path PubPathCircle12;
    nav_msgs::Path PubPathCircle21;
    nav_msgs::Path PubPathCircle22;
};

inline Eigen::Vector2d tydubins::GetCentreCircle(const pointnode node,const bool left) const{
    Eigen::Vector2d centre;

    if(left){
        centre[0] = node.x + mMinra*cos(node.angle + M_PI/2.0);
        centre[1] = node.y + mMinra*sin(node.angle + M_PI/2.0);
    }
    else{
        centre[0] = node.x + mMinra*cos(node.angle - M_PI/2.0);
        centre[1] = node.y + mMinra*sin(node.angle - M_PI/2.0);

    }
    // std::cout << "centre = " << centre << std::endl;
    return centre;
}

inline double tydubins::Cul_distance(const Eigen::Vector2i point1,const Eigen::Vector2i point2) const{
    return resolution*(point1 - point2).norm();
}

inline double tydubins::Cul_triangle_cos(const Eigen::Vector2d point1,const Eigen::Vector2d point2,const Eigen::Vector2d point3) const{

    double l1 = (point2 - point3).norm();
    double l2 = (point1 - point2).norm();
    double l3 = (point1 - point3).norm();
    return acos((l2*l2 + l3*l3 - l1*l1) / (2.0*l2*l3));
}

inline bool tydubins::Index2Coord(const Eigen::Vector2i index, geometry_msgs::PoseStamped &coord) const
{
    double x = (index[0]) * resolution + MAP.info.origin.position.x;
    double y = (index[1]) * resolution + MAP.info.origin.position.y;

    coord.pose.position.x = x;
    coord.pose.position.y = y;
    coord.pose.position.z = 0;
    return true;
};

inline bool tydubins::Is_obstacle(const Eigen::Vector2d obscoord) const{
    int x = (obscoord[0] - MAP.info.origin.position.x) / resolution;
    int y = (obscoord[1] - MAP.info.origin.position.y) / resolution;
    // std::cout << "Is_obstacle x== " << x << std::endl;
    // std::cout << "Is_obstacle y== " << y << std::endl;
    int index = y *  map_index_x + x;
    int cost = MAP.data[index]; //地图的cost为非整数，因此先进行类型转换。

    if(index > map_index_x * map_index_y)
        return false;
    
    return ( cost != 0);
};


#endif

