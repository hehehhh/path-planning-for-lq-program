#ifndef _STANDARD_PATH_H_
#define _STANDARD_PATH_H_

#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include "geometry_msgs/PoseStamped.h"
#include "path_search/dubins.h"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cmath>

class stard_path
{
private:
    double mradius = 400; 
    int pathlength = 100;
    //同名函数，共同作用。
    void relativePath(nav_msgs::Path &path, std::vector<double> odom);
    void GetTopath(nav_msgs::Odometry odom, nav_msgs::Path path,int type);
    //开始定位的时刻的odom，不进行更改。
    nav_msgs::Odometry localizationodom;
    //获取相对于定位开始时刻的值的行走路径，也就是说，只能传入localizationodom
    void GetTopath(nav_msgs::Odometry odom);
    //获取局部路径，传入odom，即实时变化的odom
    void GetlocalPath(const nav_msgs::Odometry &odom,nav_msgs::Path &localOutpath, int type);
    //从另一路径上截取某一局部路径。
    nav_msgs::Path returnLocalPath(nav_msgs::Path globalpath,int starttindex, int length);
    //四元素转欧拉角
    inline Eigen::Vector3d Quataition2Euler(const geometry_msgs::Quaternion ge_quat)const;
    
    nav_msgs::Path circle_path;
    nav_msgs::Path line_path;
    nav_msgs::Path Octagonal_path;

    bool localizationflag = 0;
    nav_msgs::Odometry odom;

    int start_circle_index;
    int start_line_index;
    int start_octagonal_index;

public:
    //外部访问的局部路径。
    nav_msgs::Path Localcircle_path;
    nav_msgs::Path Localline_path;
    nav_msgs::Path LocalOctagonal_path;
    //完整路径
    nav_msgs::Path Outcircle_path;
    nav_msgs::Path Outline_path;
    nav_msgs::Path OutOctagonal_path;

    stard_path();
    ~stard_path();
    //获取相对于某相对位置的值。
    void relativePath(std::vector<double> odompt);

    void OdomCB(const nav_msgs::Odometry msg);

};

inline Eigen::Vector3d stard_path::Quataition2Euler(const geometry_msgs::Quaternion ge_quat) const{
    Eigen::Vector3d euler;

    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(ge_quat, tf_quat);

    tf::Matrix3x3(tf_quat).getRPY(euler[0], euler[1], euler[2]);
    return euler;
}

#endif