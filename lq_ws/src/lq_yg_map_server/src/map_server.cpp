#include <lq_yg_map_server/map_server.h>

map_server::map_server()
{
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = 0.383;
    orientation.w = 0.924;

    // orientation.x = 0.0;
    // orientation.y = 0.0;
    // orientation.z = 0.0;
    // orientation.w = 1.0;
}

map_server::~map_server()
{
    
}

void map_server::init(ros::NodeHandle &nh){
    //subscribes
    mTaskOcMapSub.subscribe(nh, "/marine_rader_map", 1);
    mTaskOcMapfilter = new tf::MessageFilter<nav_msgs::OccupancyGrid>(mTaskOcMapSub, tfListener,"map",100);
    mTaskOcMapfilter->registerCallback(boost::bind(&map_server::FromTaskOcMapfilterCB, this,_1));

    mRadarMapSub = nh.subscribe("/marine_radar_static_obs", 1, &map_server::FromRadarMapCB, this);
    mCameraMapSub = nh.subscribe("/camera_static_obs", 1, &map_server::FromCameraMapCB, this);
    
    // mOdomSub = nh.subscribe("odom", 1, &map_server::FromOdomCB, this);


    //server
    mServer = nh.advertiseService("map_clear",&map_server::doReq,this);


    //publishers
    mOutputMapPub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
    mHeartBeatPub = nh.advertise<std_msgs::String>("/map_server_heartbeat", 1);

    mOutputMap = open_map();
    mInfationMap = mOutputMap;
    std::cout << "mOutputMap.info.width  = "<< mInfationMap.info.width << std::endl;
    std::cout << "mOutputMap.info.height  = "<< mInfationMap.info.height << std::endl;
    while(ros::ok()){
        ros::spinOnce();
        if(flagTaskOcMap == 1){
            task2putmap(mFromTaskOcMap,mOutputMap);
            flagTaskOcMap = 0;
        }
        
        // if(flagRadarMap == 1){
        //     if(radar2putmap(mFromRadarMap,mOutputMap)){
        //         flagRadarMap = 0;
        //     }
        // }
        // if(flagCameraMap == 1){
        //     if(camera2putmap(mFromCameraMap,mOutputMap)){
        //         flagCameraMap = 0;
        //     }
        // }
        mOutputMapPub.publish(mInfationMap);
        // std::cout << "Outputmap success !! "<< std::endl;

        current_time = ros::Time::now();
        // ss << msg_front << std::fixed << std::setprecision(6) << current_time.toSec() << "s";
        mHeartBeatmsg.data = ss.str();

        mHeartBeatPub.publish(mHeartBeatmsg);
        // ROS_INFO("Current time = %.8f",current_time.toSec());
        // std::cout << ss.str() << std::endl;
        // ss.str("");
        rate1.sleep();
    }
}

// void map_server::FromTaskOcMapCB(const nav_msgs::OccupancyGrid &msg){
//     mFromTaskOcMap = msg;
//     mFromTaskOcMapResolution = mFromTaskOcMap.info.resolution;
//     flagTaskOcMap = 1;
// }

void map_server::FromTaskOcMapfilterCB(const nav_msgs::OccupancyGridConstPtr &msg){
    mFromTaskOcMap = *msg;
    mFromTaskOcMapResolution = mFromTaskOcMap.info.resolution;
    flagTaskOcMap = 1;

}
void map_server::FromRadarMapCB(const visualization_msgs::MarkerArray &msg){
    mFromRadarMap = msg;
    // std::cout << "callback success " <<std::endl;
    radar2putmap(mFromRadarMap,mOutputMap);
    flagRadarMap = 1;
}

void map_server::FromCameraMapCB(const visualization_msgs::MarkerArray &msg){
    mFromCameraMap = msg;

    flagCameraMap = 1;
}

// void map_server::FromOdomCB(const nav_msgs::Odometry &msg){
//     orientation = msg.pose.pose.orientation;
//     marin_originx = msg.pose.pose.position.x;
//     marin_originy = msg.pose.pose.position.y;
// }

bool map_server::doReq(lq_yg_map_server::ClearSubMap::Request& req,
        lq_yg_map_server::ClearSubMap::Response& resp)
    {
        mOutputMap = open_map();
        mInfationMap = inflateMap(mOutputMap);
        mOutputMapPub.publish(mInfationMap);
        // ros::Rate rate(2);
        // rate.sleep();
        current_time = ros::Time::now(); std::stringstream ss;
        ss << std::string("map_clear_heartbeat: ") << std::fixed << std::setprecision(6) << current_time.toSec() << "s";
        resp.message = ss.str();
        resp.success = true;
        return true;
    }

bool map_server::task2putmap(const nav_msgs::OccupancyGrid &taskmap, nav_msgs::OccupancyGrid &mOutputMap){
    
    mOutputMap.header = taskmap.header;
    mOutputMap.header.frame_id = "map";

    tf::StampedTransform radermapTomapTf;
    tf::Quaternion qua;
    try{
        tfListener.lookupTransform("map",taskmap.header.frame_id,taskmap.header.stamp,radermapTomapTf);
        // std::cout << "task2putmaptask2putmaptask2putmap taskmap.header.stamp = " << taskmap.header.stamp << std::endl;
    }
    catch(tf::TransformException &ex){
        ROS_ERROR_STREAM("Transform error of sensor data:" << ex.what() << ", quitting callback");
        return false;
    }
    qua = radermapTomapTf.getRotation();

    orientation.x = qua.getX();
    orientation.y = qua.getY();
    orientation.z = qua.getZ();
    orientation.w = qua.getW();
    marin_originx = radermapTomapTf.getOrigin().getX();
    marin_originy = radermapTomapTf.getOrigin().getY();

    // std::cout << "orientation = " << orientation << std::endl;
    // std::cout << "marin_originx = " << marin_originx << std::endl;
    // std::cout << "marin_originy = " << marin_originy << std::endl;

    Eigen::Vector3d euler;
    euler = Quataition2Euler(orientation);
    
    double alpha1 = euler[2];
    double alpha2 = 0;
    // std::cout << "task2putmap success "  << std::endl;

    int index1;int index2;
    for(int y = 0; y < taskmap.info.height; ++y)
        for(int x = 0; x < taskmap.info.width; ++x){
            double realy = y - taskmap.info.height/2.0;
            double realx = x - taskmap.info.width/2.0;
            alpha2 = atan2(realy,realx);
            double L = taskmap.info.resolution * sqrt(realx*realx + realy*realy) ;
            double tempx = L*cos(alpha1 + alpha2)  + marin_originx ;
            double tempy = L*sin(alpha1 + alpha2)  + marin_originy ;
            // double tempx = L*cos(alpha1 + alpha2) + marin_originx;
            // double tempy = L*sin(alpha1 + alpha2) + marin_originy;

            int X = (tempx  - mOutputMap.info.origin.position.x)/mOutputMap.info.resolution; // Occupanymap的xy为整形，需转成型后向下取整，后者为坐标系相对位置
            int Y = (tempy  - mOutputMap.info.origin.position.y)/mOutputMap.info.resolution;
            // std::cout << "X = " << Y <<std::endl;
            // std::cout << "Y = " << Y <<std::endl;
            if(X >= mOutputMap.info.width || Y >= mOutputMap.info.height)
                continue;
            index1 = y * taskmap.info.width + x;
            index2 = Y * mOutputMap.info.width + X;
            // std::cout << "index1 = " << index1 <<std::endl;
            // std::cout << "index2 = " << index2 <<std::endl;

            if(mOutputMap.data[index2] == 99){
                continue;
            }

            mOutputMap.data[index2] = taskmap.data[index1];
        };
    mInfationMap = inflateMap(mOutputMap);
    return true;
}

bool map_server::radar2putmap(const visualization_msgs::MarkerArray &radarmap, nav_msgs::OccupancyGrid &mOutputMap){

    if(radarmap.markers.empty()){
        return false;
    }

    tf::StampedTransform radermapTomapTf;
    tf::Quaternion qua;
    try{
        tfListener.lookupTransform("map",mFromTaskOcMap.header.frame_id,radarmap.markers[0].header.stamp,radermapTomapTf);
        // std::cout << "taskmap.header.stamp = " << radarmap.markers[0].header.stamp<< std::endl;

    }
    catch(tf::TransformException &ex){
        ROS_ERROR_STREAM("Transform error of sensor data:" << ex.what() << ", quitting callback");
        return false;
    }
    
    qua = radermapTomapTf.getRotation();
    orientation.x = qua.getX();
    orientation.y = qua.getY();
    orientation.z = qua.getZ();
    orientation.w = qua.getW();
    marin_originx = radermapTomapTf.getOrigin().getX();
    marin_originy = radermapTomapTf.getOrigin().getY();

    Eigen::Vector3d euler;
    euler = Quataition2Euler(orientation);
    double alpha1 = euler[2];
    double alpha2 = 0;

    // alpha1 = M_PI/6.0;
    // alpha2 = 0;
    // std::cout << "radar2putmap success "  << std::endl;

    for(int i = 0; i < radarmap.markers.size(); ++i){
        double realy = radarmap.markers[i].pose.position.y;
        double realx = radarmap.markers[i].pose.position.x;
        // std::cout << "realy = "  << realy <<std::endl;
        // std::cout << "realx = "  << realx <<std::endl;
        alpha2 = atan2(realy,realx);

        double L = sqrt(realx*realx + realy*realy) ;
        // std::cout << "L = "  << L <<std::endl;
        double tempx = L*cos(alpha1 + alpha2)  + marin_originx ;
        double tempy = L*sin(alpha1 + alpha2)  + marin_originy ;
    
        int X = (tempx  - mOutputMap.info.origin.position.x)/mOutputMap.info.resolution;// 双值都为float64，直接相加向下取整，后者为坐标系相对位置
        int Y = (tempy  - mOutputMap.info.origin.position.y)/mOutputMap.info.resolution;
        // std::cout << "X = "  << X <<std::endl;
        // std::cout << "Y = "  << Y <<std::endl;

        // int z = obstacle.points[0].z / 5.0;
        int width = mOutputMap.info.width;
        int height = mOutputMap.info.height;
        if(X >= mOutputMap.info.width || X >= mOutputMap.info.height){
            continue;
        }
        int index;
        std::vector<int> temppoints;
        index = (Y) *mOutputMapleng_x + (X);
        mOutputMap.data[index] = 99;
        // if(trigger == 20 || trigger == 0){
        //     CleadObsPoint.clear();
        //     if(trigger == 20)
        //         CleadObsPoint.push_back(temppoints);
        //     else{
        //         // clearMap()/;
        //         CleadObsPoint.push_back(temppoints);
        //         trigger = 1;
        //     }
        //     // std::cout << "sss" <<std::endl;
        // }

        // std::cout << "trigger == " << trigger <<std::endl;

        // int index = y * mOutputMapleng_x + x;
        // mOutputMap.data[index] = 100;

    }
    mInfationMap = inflateMap(mOutputMap);
    // trigger--;
    return true;
}

bool map_server::camera2putmap(const visualization_msgs::MarkerArray &cameramap, nav_msgs::OccupancyGrid &mOutputMap){

    if(cameramap.markers.empty()){
        return false;
    }
    Eigen::Vector3d euler;
    euler = Quataition2Euler(orientation);
    double alpha1 = euler[2];
    double alpha2 = 0;
    for(int i = 0; i < cameramap.markers.size(); ++i){
        double realy = cameramap.markers[i].pose.position.y;
        double realx = cameramap.markers[i].pose.position.x;
        // std::cout << "realy = "  << realy <<std::endl;
        // std::cout << "realx = "  << realx <<std::endl;
        alpha2 = atan2(realy,realx);

        double L = sqrt(realx*realx + realy*realy) ;
        double tempx = L*cos(alpha1 + alpha2)  + marin_originx ;
        double tempy = L*sin(alpha1 + alpha2)  + marin_originy ;
    
        int X = (tempx  - mOutputMap.info.origin.position.x)/mOutputMap.info.resolution;// 双值都为float64，直接相加向下取整，后者为坐标系相对位置
        int Y = (tempy  - mOutputMap.info.origin.position.y)/mOutputMap.info.resolution;

        // int z = obstacle.points[0].z / 5.0;
        if(X > mOutputMap.info.width || X > mOutputMap.info.height){
            continue;
        }
        int index;
        std::vector<int> temppoints;
        index = (Y) *mOutputMapleng_x + (X);
        mOutputMap.data[index] = 99;
        temppoints.push_back(X);
        temppoints.push_back(Y);
        CleadObsPoint.push_back(temppoints);
        std::cout << "CleadObsPoint.size() = " << CleadObsPoint.size() <<std::endl;

        if(CleadObsPoint.size() == 21){
            clearMap();
            // std::cout << "CleadObsPoint = " << CleadObsPoint[0][0] <<std::endl;
            // std::cout << "CleadObsPoint = " << CleadObsPoint[0][1] <<std::endl;

            for(int before = 0; before < CleadObsPoint.size() - 1; ++before){
                CleadObsPoint[before][0] = CleadObsPoint[before + 1][0];
                CleadObsPoint[before][1] = CleadObsPoint[before + 1][1];
            }
            CleadObsPoint.erase(CleadObsPoint.end());
            // std::cout << "sss" <<std::endl;
        }
        // std::cout << "sss" <<std::endl;

        // int index = y * mOutputMapleng_x + x;
        // mOutputMap.data[index] = 100;

    }

    mInfationMap = inflateMap(mOutputMap);
    trigger--;
    return true;

}

nav_msgs::OccupancyGrid map_server::inflateMap(const nav_msgs::OccupancyGrid &map){
    nav_msgs::OccupancyGrid ansmap;
    ansmap = map;

    int width = ansmap.info.width;
    int height = ansmap.info.height;

    for(int i = 0; i < width; ++i)
        for(int j = 0; j < height; ++j){
            int index = j * width + i;
            if(ansmap.data[index] != 100 && ansmap.data[index] != 99)
                continue;
            
            for(int dx = -mflationgridnum; dx <= mflationgridnum; dx +=1){
                for(double dy = -mflationgridnum; dy < mflationgridnum; dy +=1){
                    int newi = i + dx;
                    int newj = j + dy;
                    if(newi >= mOutputMap.info.width || newj >= mOutputMap.info.height){
                        continue;
                    }
                    int newindex = newj * width + newi;

                    if(ansmap.data[newindex] == 100)
                        continue;
                    ansmap.data[newindex] = 50;
                }
            }

        }
    
    return ansmap;

}

void map_server::clearMap(){
    for(int i = 0; i < CleadObsPoint.size(); ++i){
        int X = CleadObsPoint[i][0];
        int Y = CleadObsPoint[i][1];
        // std::cout << "X = " << X <<std::endl;
        // std::cout << "Y = " << Y <<std::endl;

        for(int dx = -mflationgridnum; dx <= mflationgridnum; dx +=1){
            for(int dy = -mflationgridnum; dy < mflationgridnum; dy +=1){
                int newi = X + dx;
                int newj = Y + dy;
                if(newi >= mOutputMap.info.width || newj >= mOutputMap.info.height){
                    continue;
                }
                int newindex = newj * mInfationMap.info.width + newi;

                mInfationMap.data[newindex] = 0;
            }
        }
    }    
    // std::cout << "清一次" << std::endl;
}

nav_msgs::OccupancyGrid map_server::open_map(){
    nav_msgs::OccupancyGrid empty_map;
    empty_map.header.frame_id = "map";
    cv::Mat map_image = cv::imread("/home/helll/copycopy/work_ws/src/lq_yg_map_server/map/blank_map.pgm",cv::IMREAD_GRAYSCALE);
    
    if(map_image.empty()){

        ROS_ERROR("Fail to open empty_map file !");
    }

    empty_map.info.height = map_image.cols;
    empty_map.info.width = map_image.rows;
    empty_map.info.resolution = mresulution;
    std::vector<int8_t> map_date(map_image.data, map_image.data + map_image.total());
    empty_map.data = map_date;
    empty_map.info.origin.position.x = -400*5;
    empty_map.info.origin.position.y = -400*5;
    empty_map.info.origin.position.z = 0;
    return empty_map;
}