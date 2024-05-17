#include "path_search/bspline_local_planner.h"

Bspline_planner::Bspline_planner(){}
Bspline_planner::~Bspline_planner(){}

void Bspline_planner::GlobalPathCB(const nav_msgs::Path &msg){
    mGlobalPath = msg;
    spline_path.header.frame_id = mGlobalPath.header.frame_id;
    // std::cout << "GlobalPathCB" << std::endl;
    mGlobalPathFlag = 1;
}
void Bspline_planner::LocalMapCB(const nav_msgs::OccupancyGrid &msg){
    mLocalMap = msg;
    // std::cout << "LocalMapCB" << std::endl;
    mLocalMapFlag = 1;
}
void Bspline_planner::OdometryCB(const nav_msgs::Odometry &msg){
    mOdom = msg;
    // std::cout << "OdometryCB" << std::endl;
    mOdomFlag = 1;
}

bool Bspline_planner::init(){
    if(mGlobalPathFlag == 0 || mLocalMapFlag == 0 || mOdomFlag == 0){
        // std::cout << "initial failed" << std::endl;
        return false;
    }
    mGlobalPathFlag = 1;
    mLocalMapFlag = 1;
    mOdomFlag = 1;
    // std::cout << "initial success !" << std::endl;
    resulotion = mLocalMap.info.resolution;
    // ros::Time t1 = ros::Time::now();
    //map init
    // for(int x = 0; x < mLocalMap.info.width; ++x)
    //     for(int y = 0; y < mLocalMap.info.height; ++y){
    //         int index = x + y * mLocalMap.info.width;
    //         int cost = mLocalMap.data[index];

    //         if(cost != 0){
    //             std::string keypt;
        
    //             keypt.push_back(x);
    //             keypt.push_back(y);
    //             mCheckMap[keypt] = false;
    //         }
    //         else{
    //             std::string keypt;
    //             keypt.push_back(x);
    //             keypt.push_back(y);
    //             mCheckMap[keypt] = true;
    //         }
    //     }
    // ros::Time t2 = ros::Time::now();
    // ROS_INFO("map init 时间为 %.8f s",t2.toSec()-t1.toSec());
    // std::cout << "map init success !" << std::endl;

    //startindex update
    if(mStartindex == 0){
        mStartindex = 1;
    }else{
        int index;
        NearedIndex(mLobalPath, mOdom, index);
        mStartindex = mStartindex + index;
        
    }
    // std::cout << "startindex update success !" << std::endl;

    return true;
}

void Bspline_planner::NearedIndex(const nav_msgs::Path &path, const nav_msgs::Odometry &odom, int &index){
    //更新StartIndex的值
    int flagii = 0;
    double minDistance = 999999.0;
    // std::cout << "path.poses.size() = "<< path.poses.size()<<std::endl;
    for(int i = 0 ; i < path.poses.size(); ++i){

        double dx = mOdom.pose.pose.position.x - path.poses[i].pose.position.x;
        double dy = mOdom.pose.pose.position.y - path.poses[i].pose.position.y;

        double distance = sqrt(dx*dx + dy*dy);
        // std::cout << "distance = "<< distance << std::endl;
        if(distance < minDistance){
            minDistance = distance;
            //获取局部路径上的点的相对位置 此时flagii在局部路径上。 即falgii只能在0-mLocalPathLength之间，不会超过全局路径。mLocalPathLength是指定的局部路径长度。
            flagii = i;
        }
    }
    index = flagii;
    // std::cout << "index = "<< index<<std::endl;

}
void Bspline_planner::GetpointOfGlobalpath(std::vector<int> Waypoints, std::vector<std::vector<double>> &ConWaypoints){
    // std::cout << "GetpointOfGlobalpath start !" << std::endl;

    if(mLobalPath.poses.size() == 0){
        for(auto i : Waypoints){
            GetpointOfGlobalpath(mGlobalPath, mStartindex, i, ConWaypoints);
        }
    }else{
        for(int i  = 0; i < 4; ++i){
            int pianyi = Waypoints[i];
            GetpointOfGlobalpath(mLobalPath, 0, pianyi, ConWaypoints);
        }
        for(int i = 4; i < Waypoints.size(); ++i){
            int pianyi = Waypoints[i];
            GetpointOfGlobalpath(mGlobalPath, mStartindex, pianyi, ConWaypoints);
        }
    }

    
}

void Bspline_planner::GetpointOfGlobalpath(const nav_msgs::Path &globalpath, int start, int pianyi, std::vector<std::vector<double>> &ConWaypoints){
    std::vector<double> pt;
    double x = globalpath.poses[start + pianyi].pose.position.x;
    double y = globalpath.poses[start + pianyi].pose.position.y;
    pt.push_back(x);
    pt.push_back(y);
    ConWaypoints.push_back(pt);
    // std::cout << "pt0 = " << pt[0] << std::endl;
    // std::cout << "pt1 = " << pt[1] << std::endl;
    //updatelocalpath
    mLobalPath.poses.clear();
    for(int i = 0; i < mLocalPathLength; ++i){
        int length = mGlobalPath.poses.size();
        int globalpathindex = std::min(mStartindex + i, length - 1);

        mLobalPath.poses.push_back(mGlobalPath.poses[globalpathindex]);

    }
    return;
}

bool Bspline_planner::SplinePathSearch(){
    ros::Time t1 = ros::Time::now();
    if(SplinePathSearch(mGlobalPath, mLocalMap, mOdom)){
        ros::Time t2 = ros::Time::now();
        // ROS_INFO("SplinePathSearch时间为 %.8f s",t2.toSec()-t1.toSec());
        // std::cout << "mStartindex = " << mStartindex << std::endl;

        return true;
    }
    else{
        // NearedIndex(mGlobalPath,mOdom,mStartindex);
        // std::cout << "mStartindex = " << mStartindex << std::endl;
    }
    return false;
}

bool Bspline_planner::SplinePathSearch(const nav_msgs::Path &globalpath, const nav_msgs::OccupancyGrid &localmap,const nav_msgs::Odometry odom){
    if(!init())
        return false;

    std::vector<std::vector<double>> Con_points;

    //将当前位置的点存储进控制点数组。
    std::vector<double> pt;
    pt.push_back(odom.pose.pose.position.x);
    pt.push_back(odom.pose.pose.position.y);
    Con_points.push_back(pt);

    //将剩余5点存入控制点数组
    std::vector<int> waypoints= {4,8,12,16,20,24};
    GetpointOfGlobalpath(waypoints, Con_points);

    // std::cout << "GetpointOfGlobalpath success !" << std::endl;
    if(!CollitionAvoid(Con_points, mLocalMap)){
        // std::cout << "CollitionAvoid failed !" << std::endl;

        return false;
    }

    // std::cout << "CollitionAvoid success !" << std::endl;
    // for(auto ii : Con_points){
    //     for(auto i : ii)
    //         std::cout << i  << " ";
    //     std::cout << std::endl;
    // }

    spline_path = bspline_path(Con_points);
    // std::cout << "bspline_path success !" << std::endl;

    return true;
}

bool Bspline_planner::CollitionAvoid(std::vector<std::vector<double>> &ConWaypoints,const nav_msgs::OccupancyGrid &map){
    //检查路径点的障碍物框是否存在障碍物
    // std::cout << "CollitionAvoid start !" << std::endl;
    // std::cout << "ConWaypoints.size() =  " << ConWaypoints.size()<< std::endl;

    for(int pointii = 0; pointii < ConWaypoints.size(); ++pointii){
        // std::cout << "pointii =  " << pointii<< std::endl;
        if(IsObstable(ConWaypoints[pointii])){
            // std::cout << "renew start !" << std::endl;

            if(!renewpoint(ConWaypoints, pointii)){
                // std::cout << "renew false !" << std::endl;
                return false;
            }
        }

    }

    // std::cout << "CollitionAvoid success !" << std::endl;
    return true;
}

bool Bspline_planner::renewpoint(std::vector<std::vector<double>> &ConWaypoints, int pointindex){
    int length = ConWaypoints.size() -1;
    if(pointindex == length || pointindex == 0){
        // std::cout << "ConWaypoints.size() -1" << std::endl;
        return true;
    }
    // std::cout << pointindex << std::endl;
    std::vector<int> checkset;
    if(mleftflag == 0)
        checkset = {-1,-2,-3,-4,1,2,3,4};
    else
        checkset = {1,2,3,4,-1,-2,-3,-4};

    double x;
    double y;
    std::vector<double> centrept;
    x = (ConWaypoints[pointindex + 1][0] + ConWaypoints[pointindex - 1][0])/2.0;
    y = (ConWaypoints[pointindex + 1][1] + ConWaypoints[pointindex - 1][1])/2.0;
    centrept.push_back(x);
    centrept.push_back(y);

    double dy = ConWaypoints[pointindex + 1][1] - ConWaypoints[pointindex - 1][1];
    double dx = ConWaypoints[pointindex + 1][0] - ConWaypoints[pointindex - 1][0];

    double angle = atan2(dy,dx) + M_PI/2.0;

    for(auto &errorSet : checkset){
        //将该点位置更新，从前后两边的中点开始更新。每个点偏移20m。
        std::vector<double> pt;

        x = centrept[0] + 1.0*errorSet*checkgridlength*cos(angle);
        y = centrept[1] + 1.0*errorSet*checkgridlength*sin(angle);

        pt.push_back(x);
        pt.push_back(y);

        if(!IsObstable(pt)){
            // std::cout << "ConWaypoints[pointindex] 0= " << ConWaypoints[pointindex][0]<<std::endl;
            // std::cout << "ConWaypoints[pointindex] 1= " << ConWaypoints[pointindex][1]<<std::endl;
            // std::cout << "pointindex= " << pointindex<<std::endl;

            ConWaypoints[pointindex] = pt;
            if(errorSet < 0)
                mleftflag = 0;
            else
                mleftflag = 1;
            // std::cout << "renew success !" << std::endl;
            return true;
        }

    }
    // std::cout << "renew false !" << std::endl;
    // std::cout << "ConWaypoints[pointindex] 0= " << ConWaypoints[pointindex][0]<<std::endl;
    // std::cout << "ConWaypoints[pointindex] 1= " << ConWaypoints[pointindex][1]<<std::endl;
    return false;
}

void Bspline_planner::expandSearch(std::vector<std::vector<double>> &points){
    std::vector<double> point1 = points[4];
    std::vector<double> point2;

    int globalstart = mStartindex + mLocalPathLength;
    for(int i = globalstart; i < mGlobalPath.poses.size(); ++i){
        point2.clear();
        point2.push_back(mGlobalPath.poses[i].pose.position.x);
        point2.push_back(mGlobalPath.poses[i].pose.position.y);\


    }
}

bool Bspline_planner::IsObstable(std::vector<double> point){
    double x = point[0];
    double y = point[1];

    checkgridnum = 3.0*checkgridlength/resulotion/2.0;

    for(double i = -checkgridnum; i <= checkgridnum; i+=1.0)
        for(double j = -checkgridnum; j <= checkgridnum; j+=1.0)
        {
            int checkx = int (x - mLocalMap.info.origin.position.x + i*resulotion)/resulotion;
            int checky = int (y - mLocalMap.info.origin.position.y + j*resulotion)/resulotion;

            if(checkx < 0 || checkx >= mLocalMap.info.width || checky < 0 || checky >= mLocalMap.info.height)
                continue;
            
            int index = checkx + checky * mLocalMap.info.width;
            int cost = mLocalMap.data[index];

            if(cost != 0){
                // std::cout << " is obstable "  << std::endl;
                return true;
            }
        }
    // std::cout << " not obstable "  << std::endl;

    return false;
}

nav_msgs::Path Bspline_planner::bspline_path(const std::vector<std::vector<double>> &points){
    Eigen::MatrixXd opt_points;
    Eigen::VectorXd point(2);
    tybspline mbspline;

    int length = points.size();
    for(int i = 0; i < length; i++){
        point[0] = points[i][0];
        point[1] = points[i][1];
        // std::cout << "point[0]  = " << point[0] << std::endl;
        // std::cout << "point[1]  = " << point[1] << std::endl;
        opt_points.conservativeResize(2, opt_points.cols() + 1);
        opt_points.col(opt_points.cols() - 1) = point;       
    }
    mbspline.init(opt_points,3);
    mbspline.setUniformKnot();

    nav_msgs::Path respath;
    respath.header = mLocalMap.header;
    double denselength = 200;

    geometry_msgs::PoseStamped temppose;
    for(double i = 0; i <= denselength; ++i){
        double u = i / denselength;
        // std::cout << "uuu = = " << u << std::endl;
    
        Eigen::VectorXd temppoint = mbspline.evaluateDeBoor(u);
        
        temppose.pose.position.x = temppoint[0];
        temppose.pose.position.y = temppoint[1];
        // std::cout << "temppose.pose.position.x  = " << temppose.pose.position.x << std::endl;
        // std::cout << "temppose.pose.position.y  = " << temppose.pose.position.y << std::endl;
        respath.poses.push_back(temppose);
    }

    return respath;
}


nav_msgs::Path Bspline_planner::GetSplinePath(){
    return spline_path;
}