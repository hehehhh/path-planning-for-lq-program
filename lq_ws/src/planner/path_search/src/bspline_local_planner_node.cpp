#include <ros/ros.h>
#include <iostream>
#include "path_search/bspline_local_planner.h"
#include <nav_msgs/Path.h>

int main(int argc, char  *argv[])
{   
    setlocale(LC_ALL,"");
    setlocale(LC_CTYPE,"zh_CN.utf8");

    ros::init(argc,argv,"bspline_local_planner_node");
    ros::NodeHandle nh;
    ROS_INFO("The program of bspline_local_planner_node is running ...");

    Bspline_planner Local_planner;

    ros::Subscriber Globalpath_Sub = nh.subscribe("/oooo/global_path", 1, &Bspline_planner::GlobalPathCB, &Local_planner);
    ros::Subscriber Localmap_Sub = nh.subscribe("/map", 1, &Bspline_planner::LocalMapCB, &Local_planner);
    ros::Subscriber Odom_Sub = nh.subscribe("/odom", 1, &Bspline_planner::OdometryCB, &Local_planner);
 
    ros::Publisher path_Pub = nh.advertise<nav_msgs::Path>("/oooo/lobal_path",1);

    // ros::Publisher path1_Pub = nh.advertise<nav_msgs::Path>("/oooo/circle1",1); //检测用，不需要。
    // ros::Publisher path2_Pub = nh.advertise<nav_msgs::Path>("/oooo/circle2",1);
    // ros::Publisher path3_Pub = nh.advertise<nav_msgs::Path>("/oooo/circle3",1);
    // ros::Publisher path4_Pub = nh.advertise<nav_msgs::Path>("/oooo/circle4",1);
    // ros::Publisher point1_maker = nh.advertise<visualization_msgs::Marker>("/oooo/point1",1);


    nav_msgs::Path path;

    while (ros::ok())
    {
        ros::spinOnce();

        if(Local_planner.SplinePathSearch()){
            path = Local_planner.GetSplinePath();
            path_Pub.publish(path);    
        }

        // path1_Pub.publish(path_search_test.PubPathCircle11);
        // path2_Pub.publish(path_search_test.PubPathCircle12);
        // path3_Pub.publish(path_search_test.PubPathCircle21);
        // path4_Pub.publish(path_search_test.PubPathCircle22);
        // point1_maker.publish(path_search_test.Make_point1);
    }
    
    return 0;
}