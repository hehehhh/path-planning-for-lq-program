#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "standard_path.h"

int main(int argc, char  *argv[])
{   
    setlocale(LC_ALL,"");
    setlocale(LC_CTYPE,"zh_CN.utf8");

    ros::init(argc,argv,"path_search_test");
    ros::NodeHandle nh;
    ROS_INFO("The program of standard_path_test is running ...");

    stard_path pathhh;
    //可以不用，手动设置即可，此处设置为0，表示没有相对位置。
    std::vector<double> ralaposition = {0,0};
    pathhh.relativePath(ralaposition);

    //订阅者，由于odom已经有默认值，因此不接受也可以进行处理，接收到后自动转换。
    ros::Subscriber Odom_Sub = nh.subscribe<nav_msgs::Odometry>("/initialpose", 1, &stard_path::OdomCB, &pathhh);
    
    ros::Publisher circle_path_Pub = nh.advertise<nav_msgs::Path>("/circle_path",1);
    ros::Publisher line_path_Pub = nh.advertise<nav_msgs::Path>("/line_path",1);
    ros::Publisher Octagonal_path_Pub = nh.advertise<nav_msgs::Path>("/Octagonal_path",1);
    
    ros::Publisher circle_path_Pub_local = nh.advertise<nav_msgs::Path>("/circle_path_local",1);
    ros::Publisher line_path_Pub_local = nh.advertise<nav_msgs::Path>("/line_path_local",1);
    ros::Publisher Octagonal_path_Pub_local = nh.advertise<nav_msgs::Path>("/Octagonal_path_local",1);

    // ros::Publisher pub1 = nh.advertise<nav_msgs::Path>("/circle1",1);
    // ros::Publisher pub2 = nh.advertise<nav_msgs::Path>("/circle2",1);
    // ros::Publisher pub3 = nh.advertise<nav_msgs::Path>("/circle3",1);
    // ros::Publisher pub4 = nh.advertise<nav_msgs::Path>("/circle4",1);

    ros::Rate rate = 1;

    while (ros::ok())
    {
        ros::spinOnce();
        circle_path_Pub.publish(pathhh.Outcircle_path);
        line_path_Pub.publish(pathhh.Outline_path);
        Octagonal_path_Pub.publish(pathhh.OutOctagonal_path);
        
        circle_path_Pub_local.publish(pathhh.Localcircle_path);
        line_path_Pub_local.publish(pathhh.Localline_path);
        Octagonal_path_Pub_local.publish(pathhh.LocalOctagonal_path);
        
        
        // pub1.publish();
        // pub1.publish();
        // pub1.publish();
        // pub1.publish();
        rate.sleep();
    }
    
    return 0;
}