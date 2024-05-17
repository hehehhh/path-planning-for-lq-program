#include "standard_path.h"
// #include "path_search/dubins.h"

stard_path::stard_path(){

    circle_path.header.frame_id = "map";
    line_path.header.frame_id = "map";
    Octagonal_path.header.frame_id = "map";
    double xxx = 1000;
    double yyy = 1100;
    //圆
    for (double iangle = 0; iangle < 2*M_PI; iangle += 0.02){
        geometry_msgs::PoseStamped temppose;
        temppose.pose.position.x = xxx + mradius*sin(iangle);
        temppose.pose.position.y = yyy -mradius + mradius*cos(iangle);
        circle_path.poses.push_back(temppose);
    }
    //直线
    for (double ilength = 0; ilength < 1000; ilength += 0.2){
        geometry_msgs::PoseStamped temppose;
        temppose.pose.position.x = xxx + ilength;
        temppose.pose.position.y = yyy;
        line_path.poses.push_back(temppose);
    }
    //八字园
    for (double iangle = 0; iangle < 2*M_PI; iangle += 0.02){
        geometry_msgs::PoseStamped temppose;
        temppose.pose.position.x = xxx + mradius*sin(iangle);
        temppose.pose.position.y = yyy-mradius + mradius*cos(iangle);
        Octagonal_path.poses.push_back(temppose);
    }
    for (double iangle = M_PI; iangle > -M_PI; iangle -= 0.02){
        geometry_msgs::PoseStamped temppose;
        temppose.pose.position.x = xxx + mradius*sin(iangle);
        temppose.pose.position.y = yyy + mradius + mradius*cos(iangle);
        Octagonal_path.poses.push_back(temppose);
    }

    localizationodom.pose.pose.position.x = 100;
    localizationodom.pose.pose.position.y = 150;
    localizationodom.pose.pose.position.z = 0;

    localizationodom.pose.pose.orientation.x = 0;
    localizationodom.pose.pose.orientation.y = 0;
    localizationodom.pose.pose.orientation.z = 0;
    localizationodom.pose.pose.orientation.w = 1;

}

stard_path::~stard_path(){}
//两个同名函数，获取相对0，0的路径。
void stard_path::relativePath(std::vector<double> odompt){

    relativePath(circle_path, odompt);
    relativePath(line_path, odompt);
    relativePath(Octagonal_path, odompt);
    //获取dubins曲线路径。
    GetTopath(localizationodom);

    GetlocalPath(localizationodom, Localcircle_path, 0);
    GetlocalPath(localizationodom, Localline_path, 1);
    GetlocalPath(localizationodom, LocalOctagonal_path, 2);
}

void stard_path::relativePath(nav_msgs::Path &path, std::vector<double> odom){

    for(int i = 0; i < path.poses.size(); ++i){
        path.poses[i].pose.position.x = path.poses[i].pose.position.x + odom[0];
        path.poses[i].pose.position.y = path.poses[i].pose.position.y + odom[1];
    }
}
//获取dubins曲线。分别对三个进行dubins曲线获取。
void stard_path::GetTopath(nav_msgs::Odometry odom){

    GetTopath(odom, circle_path,0);
    GetTopath(odom, line_path,1);
    GetTopath(odom, Octagonal_path,2);

}

void stard_path::GetTopath(nav_msgs::Odometry odom, nav_msgs::Path path,int type){
    tydubins dubins_pure;

    pointnode start;
    pointnode end;
    //起点获取
    start.x = odom.pose.pose.position.x;
    start.y = odom.pose.pose.position.y;
    Eigen::Vector3d euler = Quataition2Euler(odom.pose.pose.orientation);
    start.angle = euler[2];
    std::cout << "start.angle = " << start.angle<< std::endl;
    //终点获取
    end.x = path.poses[0].pose.position.x;
    end.y = path.poses[0].pose.position.y;
    double dy = path.poses[1].pose.position.y - path.poses[0].pose.position.y;
    double dx = path.poses[1].pose.position.x - path.poses[0].pose.position.x;
    end.angle = atan2(dy, dx);
    std::cout << "end.angle = " << end.angle<< std::endl;

    nav_msgs::Path anspath;
    nav_msgs::Path dubinspath;
    //获取该起点终点下的dubins曲线。
    if (dubins_pure.PureDubins(start, end)){
        dubinspath = dubins_pure.GetdubinsPath();
    }
    
    for(int i = 0; i < dubinspath.poses.size(); ++i)
        anspath.poses.push_back(dubinspath.poses[i]);

    for(int i = 0; i < path.poses.size(); ++i)
        anspath.poses.push_back(path.poses[i]);

    anspath.header = path.header;

    if(type == 0){
        Outcircle_path = anspath;
        Localcircle_path = returnLocalPath(anspath, 0,pathlength);
        start_circle_index = 0;
    }

    if(type == 1){
        Outline_path = anspath;
        Localline_path = returnLocalPath(anspath, 0,pathlength);
        start_line_index = 0;
    }

    if(type == 2){
        OutOctagonal_path = anspath;
        LocalOctagonal_path = returnLocalPath(anspath, 0,pathlength);
        start_octagonal_index = 0;
    }

}

void stard_path::GetlocalPath(const nav_msgs::Odometry &odom, nav_msgs::Path &localOutpath, int type){

    int *startindex;
    switch (type)
    {
    case 0:
        //此时此刻，由于所有的startindex都属于取址行为，也就是说，每次的*startindex都会对start_circle_index，start_line_index，start_octagonal_index的
        //值进行改变。
        startindex = &start_circle_index;
        break;
    case 1:
        startindex = &start_line_index;
        break;
    case 2:
        startindex = &start_octagonal_index;
        break;
    }
    double mindistance = 999999;
    int flagii = 0;
    for(int i = 0; i < pathlength; i++){
        double dx = odom.pose.pose.position.x - localOutpath.poses[i].pose.position.x;
        double dy = odom.pose.pose.position.y - localOutpath.poses[i].pose.position.y;

        double distance = sqrt(dx*dx + dy*dy);
        // std::cout << (distance < mindistance) << std::endl;
        if(distance < mindistance){
            mindistance = distance;
            //获取局部路径上的点的相对位置 此时flagii在局部路径上。 即falgii只能在0-pathlength之间，不会超过全局路径。pathlength是指定的局部路径长度。
            flagii = i;
        }
    }
    //此时start存储的是每个局部路径起点在全局路径上的起点，因此在加完索引之后，就是对应的全局路径的索引值（此时startindex + flagii）。也就是全局索引 = 局部起点 + 局部偏移量 。

    *startindex = *startindex + flagii;
    
    if(type == 0){
        localOutpath = returnLocalPath(Outcircle_path, *startindex,pathlength);
    }
    if(type == 1){
        localOutpath = returnLocalPath(Outline_path, *startindex,pathlength);

    }
    if(type == 2){
        localOutpath = returnLocalPath(OutOctagonal_path, *startindex,pathlength);
    
    }
}

nav_msgs::Path stard_path::returnLocalPath(nav_msgs::Path globalpath,int starttindex, int length){
    nav_msgs::Path ansPath;
    //获取对应局部路径在全局路径上的点，即，在 
    int endindex = std::min(int(globalpath.poses.size() - 1), starttindex + length);

    ansPath.header = globalpath.header;
    for(int i = 0; i < length; ++i){
        if(starttindex + i > endindex)
            continue;
        geometry_msgs::PoseStamped temppose;
        temppose = globalpath.poses[starttindex + i];
        ansPath.poses.push_back(temppose);
    }
    return ansPath;
}

void stard_path::OdomCB(const nav_msgs::Odometry msg){
    //只有第一次进入回调函数的时候，会进行定位信息传递。
    if(localizationflag == 0){
        localizationodom = msg;
        GetTopath(localizationodom);
        odom = msg;
        localizationflag = 1;
        std::cout<< "first time" <<std::endl;
    }
    //往后都只更新局部路径。
    else{
        odom = msg;
        //进行局部路径更新
        GetlocalPath(odom, Localcircle_path, 0);
        GetlocalPath(odom, Localline_path, 1);
        GetlocalPath(odom, LocalOctagonal_path, 2);
    }

}
