#include <ros/ros.h>
#include <iostream>
#include <path_search/path_search.h>
#include <nav_msgs/Path.h>


geometry_msgs::PoseStamped Startpose,Endpose; 
void StartposeCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg ){
    Startpose.header = msg->header;
    Startpose.pose = msg->pose.pose;
    
    ROS_INFO("startpose 已经接收 ");
    std::cout << "x =  " << Startpose.pose.position.x << std::endl;
    std::cout << "y =  " << Startpose.pose.position.y << std::endl;
    std::cout << "z =  " << Startpose.pose.position.z << std::endl;

}

void EndposeCB(const geometry_msgs::PoseStampedConstPtr &msg){
    Endpose = *msg;
}


int main(int argc, char  *argv[])
{   
    setlocale(LC_ALL,"");
    setlocale(LC_CTYPE,"zh_CN.utf8");

    ros::init(argc,argv,"path_search_test");
    ros::NodeHandle nh;
    ROS_INFO("The program of path_search_test is running ...");

    path_search path_search_test;

    ros::Subscriber Startpose_Sub = nh.subscribe("/initialpose", 1, &StartposeCB);
    ros::Subscriber Occumap_Sub = nh.subscribe("/map", 1, &path_search::OccumapCB, &path_search_test);
    ros::Subscriber Endpose_Sub = nh.subscribe("/move_base_simple/goal", 1, &EndposeCB);
 
    ros::Publisher path_Pub = nh.advertise<nav_msgs::Path>("/oooo/global_path",1);

    // ros::Publisher path1_Pub = nh.advertise<nav_msgs::Path>("/oooo/circle1",1); //检测用，不需要。
    // ros::Publisher path2_Pub = nh.advertise<nav_msgs::Path>("/oooo/circle2",1);
    // ros::Publisher path3_Pub = nh.advertise<nav_msgs::Path>("/oooo/circle3",1);
    // ros::Publisher path4_Pub = nh.advertise<nav_msgs::Path>("/oooo/circle4",1);
    // ros::Publisher point1_maker = nh.advertise<visualization_msgs::Marker>("/oooo/point1",1);


    nav_msgs::Path path;

    ros::Rate rate(0.5);

    while (ros::ok())
    {
        ros::spinOnce();
        std::vector<waypoint> points;
        waypoint point;
        point.x = Startpose.pose.position.x;
        point.y = Startpose.pose.position.y;
        points.push_back(point);
        point.x = Endpose.pose.position.x;
        point.y = Endpose.pose.position.y;
        points.push_back(point);

        point.x = 700;
        point.y = 700;
        points.push_back(point);
        point.x = 1200;
        point.y = 1000;
        points.push_back(point);
        path_search_test.Getpathmain(path);
        // path_search_test.Getpathmutiple(points, path,Startpose.pose.orientation);
        
        if(!path_search_test.Getpathmutiple(points, path,Startpose.pose.orientation)){
        }

        for(int i = 0; i < path.poses.size(); ++i){
            std::cout << path.poses[i].pose.position<<std::endl;
        }
        path_Pub.publish(path);    
        
        // path1_Pub.publish(path_search_test.PubPathCircle11);
        // path2_Pub.publish(path_search_test.PubPathCircle12);
        // path3_Pub.publish(path_search_test.PubPathCircle21);
        // path4_Pub.publish(path_search_test.PubPathCircle22);
        // point1_maker.publish(path_search_test.Make_point1);
        rate.sleep();
    }
    
    return 0;
}