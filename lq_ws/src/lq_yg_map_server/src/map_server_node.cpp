#include <ros/ros.h>
#include <iostream>
#include <lq_yg_map_server/map_server.h>

int main(int argc, char  *argv[])
{   
    setlocale(LC_ALL,"");
    setlocale(LC_CTYPE,"zh_CN.utf8");

    ros::init(argc,argv,"lq_map_server");
    ros::NodeHandle nh;

    map_server map_server_node;
    map_server_node.init(nh);
    ROS_INFO("The program of lq_yg_map_server is running ...");
    return 0;
}