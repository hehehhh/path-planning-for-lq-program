#ifndef MAP_SERVER_H_
#define MAP_SERVER_H_

#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <sstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <eigen3/Eigen/Eigen>
#include <tf/tf.h>
#include "lq_yg_map_server/ClearSubMap.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#define MAX_COST 255

class map_server
{
private:
    /* data */
    ros::Rate rate1 = 1;
    ros::Rate rate2 = 0.2;
    ros::Time current_time;

    int trigger = 20; 
    int mOutputMapleng_x = 800; int mOutputMapleng_y = 800; int mresulution = 5;
    int occupied_cost = 255;
    int mflationgridnum = 5;

    double marin_originx = 50;
    double marin_originy = 50;
    geometry_msgs::Quaternion orientation;
    nav_msgs::OccupancyGrid mFromTaskOcMap;
    double mFromTaskOcMapResolution = 1.0;
    visualization_msgs::MarkerArray mFromRadarMap;
    visualization_msgs::MarkerArray mFromCameraMap;
    std::stringstream ss; std::string msg_front = "map_server_heartbeat: ";

    std_msgs::String mHeartBeatmsg;

    bool flagTaskOcMap = 0; bool flagRadarMap = 0; bool flagCameraMap = 0;
    message_filters::Subscriber<nav_msgs::OccupancyGrid> mTaskOcMapSub;
    ros::Subscriber mRadarMapSub;
    ros::Subscriber mCameraMapSub;
    // ros::Subscriber mOdomSub;

    tf::TransformListener tfListener;
    tf::MessageFilter<nav_msgs::OccupancyGrid> * mTaskOcMapfilter;
    ros::ServiceServer mServer;

    std::vector<std::vector<int>> CleadObsPoint;
    nav_msgs::OccupancyGrid mOutputMap;
    nav_msgs::OccupancyGrid mInfationMap;
    ros::Publisher mOutputMapPub;
    ros::Publisher mHeartBeatPub;

    // void FromTaskOcMapCB(const nav_msgs::OccupancyGrid &msg);
    void FromTaskOcMapfilterCB(const nav_msgs::OccupancyGridConstPtr &msg);

    void FromRadarMapCB(const visualization_msgs::MarkerArray &msg);
    void FromCameraMapCB(const visualization_msgs::MarkerArray &msg);
    // void FromOdomCB(const nav_msgs::Odometry &msg);

    nav_msgs::OccupancyGrid inflateMap(const nav_msgs::OccupancyGrid &map);

    bool doReq(lq_yg_map_server::ClearSubMap::Request& req,
          lq_yg_map_server::ClearSubMap::Response& resp);

    bool task2putmap(const nav_msgs::OccupancyGrid &taskmap, nav_msgs::OccupancyGrid &mOutputMap);
    bool radar2putmap(const visualization_msgs::MarkerArray &radarmap, nav_msgs::OccupancyGrid &mOutputMap);
    bool camera2putmap(const visualization_msgs::MarkerArray &cameramap, nav_msgs::OccupancyGrid &mOutputMap);
    nav_msgs::OccupancyGrid open_map();
    void clearMap();
    inline Eigen::Vector3d Quataition2Euler(const geometry_msgs::Quaternion ge_quat)const;

public:
    map_server();
    ~map_server();
    void init(ros::NodeHandle &nh);
};

inline Eigen::Vector3d map_server::Quataition2Euler(const geometry_msgs::Quaternion ge_quat) const{
    Eigen::Vector3d euler;

    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(ge_quat, tf_quat);

    tf::Matrix3x3(tf_quat).getRPY(euler[0], euler[1], euler[2]);
    return euler;
}

#endif