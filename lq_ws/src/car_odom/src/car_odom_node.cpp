#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "car_odom/car_odom.h"

static nav_msgs::Path global_path;
static nav_msgs::Path lobal_path;
static bool flag = 0;
static nav_msgs::Odometry odom;

void GlobalPathCB(const nav_msgs::Path &msg){
    if(msg.poses.size() == 0)
        return;
    
    if(flag == 0){
        global_path = msg;
        flag = 1;
        odom.header.frame_id = msg.header.frame_id;
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = 0.0;
        odom.pose.pose.orientation.w = 1.0;
        odom.pose.pose.position.x = global_path.poses[0].pose.position.x;
        odom.pose.pose.position.y = global_path.poses[0].pose.position.y;
        odom.pose.pose.position.z = 0;
        std::cout << " odom publish !"<< std::endl;
    }
}

void lobalPathCB(const nav_msgs::Path &msg){
    lobal_path = msg;
}

int main(int argc, char  *argv[])
{   
    setlocale(LC_ALL,"");
    setlocale(LC_CTYPE,"zh_CN.utf8");

    ros::init(argc,argv,"car_odom_node");
    ros::NodeHandle nh;
    ROS_INFO("The program of car_odom_node is running ...");

    car_model test_car;


    ros::Subscriber Globalpath_Sub = nh.subscribe("/oooo/global_path", 1, &GlobalPathCB);
    ros::Subscriber Lobalpath_Sub = nh.subscribe("/oooo/lobal_path", 1, &lobalPathCB);
    // ros::Subscriber Localmap_Sub = nh.subscribe("/local_map", 1, &Bspline_planner::LocalMapCB, &Local_planner);
    // ros::Subscriber Odom_Sub = nh.subscribe("/odom", 1, &Bspline_planner::OdometryCB, &Local_planner);
 
    ros::Publisher odom_Pub = nh.advertise<nav_msgs::Odometry>("/odom",1);

    // ros::Publisher path1_Pub = nh.advertise<nav_msgs::Path>("/oooo/circle1",1); //检测用，不需要。
    // ros::Publisher path2_Pub = nh.advertise<nav_msgs::Path>("/oooo/circle2",1);
    // ros::Publisher path3_Pub = nh.advertise<nav_msgs::Path>("/oooo/circle3",1);
    // ros::Publisher path4_Pub = nh.advertise<nav_msgs::Path>("/oooo/circle4",1);
    // ros::Publisher point1_maker = nh.advertise<visualization_msgs::Marker>("/oooo/point1",1);


    ros::Rate rate = 5;
    while (ros::ok())
    {
        ros::spinOnce();
        odom_Pub.publish(odom);

        if(lobal_path.poses.size() != 0){
            odom.pose.pose.position.x = lobal_path.poses[2].pose.position.x;
            odom.pose.pose.position.y = lobal_path.poses[2].pose.position.y;
        }
        std::cout << " odom.x = "<<odom.pose.pose.position.x <<std::endl;
        std::cout << " odom.y = "<<odom.pose.pose.position.y <<std::endl;
        rate.sleep();
    }
    
    return 0;
}