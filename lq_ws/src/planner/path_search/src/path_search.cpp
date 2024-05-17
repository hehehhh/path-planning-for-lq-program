#include <ros/ros.h>
#include "path_search/path_search.h"

path_search::path_search() {
    NodeMap = new GridNodePtr *[800];
    for (int i = 0; i < 800; i++)
    {
        NodeMap[i] = new GridNodePtr[800];
        for (int j = 0; j < 800; j++)
        {
            NodeMap[i][j] = new GridNode;
            NodeMap[i][j]->index[0] = i;
            NodeMap[i][j]->index[1] = j;
        }
    }
    // ROS_INFO("Nodemap空间 已经初始化 ");

}

path_search::~path_search() {}

bool path_search::Getpathmain(nav_msgs::Path &path)
{

    if (start_flag && end_flag && occumap_flag)
    {
        // start_flag = false;
        // end_flag = false;
        // occumap_flag = false;
        // path_flag = false;

        std::vector<waypoint> points;
        waypoint point;
        point.x = Startpose.pose.position.x;
        point.y = Startpose.pose.position.y;
        points.push_back(point);
        point.x = Endpose.pose.position.x;
        point.y = Endpose.pose.position.y;
        points.push_back(point);

        // point.x = 700;
        // point.y = 700;
        // points.push_back(point);
        // point.x = 1200;
        // point.y = 1000;
        // points.push_back(point);

        int flag = 1;
        while (flag == 1)
        {
            if(Getpathmutiple(points, path,Startpose.pose.orientation)){
               flag = 0; 

            }

        }
        
        return true;            
    }
    return false;
}

bool path_search::Getpathmutiple(const std::vector<waypoint> &points, nav_msgs::Path &path, const geometry_msgs::Quaternion &quaAngle){

    // start_flag = false;
    // end_flag = false;
    // occumap_flag = false;
    // path_flag = false;
    
    nav_msgs::Path anspath;
    ros::Time t1 = ros::Time::now();
    geometry_msgs::PoseStamped temppose;
    temppose.pose.position.x = points[0].x;
    temppose.pose.position.y = points[0].y;

    anspath.poses.push_back(temppose);

    if(!astarOneshot(points,anspath))
        return false;
    ros::Time t2 = ros::Time::now();
    ROS_INFO("astarOneshot时间为 %.8f s",t2.toSec()-t1.toSec());

    geometry_msgs::PoseStamped replanPose;
    replanPose.header = Startpose.header;
    replanPose.pose.position.x = points[0].x;
    replanPose.pose.position.y = points[0].y;
    replanPose.pose.orientation = quaAngle;
    // std::cout << "astarOneshot时间为points[points.size() - 1].x = "<<anspath.poses[anspath.poses.size() - 1].pose.position.x <<std::endl;
    // std::cout << "astarOneshot时间为points[points.size() - 1].y = "<<anspath.poses[anspath.poses.size() - 1].pose.position.y <<std::endl;
    // if(!replan(replanPose,anspath,Cost_Map))
    //     ROS_WARN("更新方位角路径失败！ ");

    for(int i = 0; i < anspath.poses.size(); ++i){
        std::cout << " before" << anspath.poses[i].pose.position<<std::endl;
    }

    // anspath = bspline_path(anspath);
    path = anspath;

    // std::cout << "points[points.size() - 1].x = "<<points[points.size() - 1].x <<std::endl;
    // std::cout << "points[points.size() - 1].x = "<<points[points.size() - 1].y <<std::endl;
    t2 = ros::Time::now();
    ROS_INFO("时间为 %.8f s",t2.toSec()-t1.toSec());
    ROS_INFO("路径生成完成！ ");

    return true;
}

bool path_search::astarOneshot(const std::vector<waypoint> &points, nav_msgs::Path &path){
    path.poses.clear();
    nav_msgs::Path anspath;
    NodePath.clear();
    anspath.header = Cost_Map.header;
    anspath.poses.clear();
    nav_msgs::Path pathorder;

    //获取第一对点，用于预防第一对dubins不能生成。
    newreplanpoint.clear();
    newreplanpoint.push_back(points[0]);
    newreplanpoint.push_back(points[1]);

    for(int i = 1; i < points.size(); ++i){
        if(!pointsinit(points[i - 1], points[i], Cost_Map));//a*算法起点终点和地图初始化

        if(!run()){
            relieveNodemap(NodeMap);
            return false;
        }

        NodePath.push_back(*current_nodeptr);
        while (current_nodeptr->index != Start_index)
        {
            current_nodeptr = current_nodeptr->cameFrom;
            NodePath.push_back(*current_nodeptr);
        }
        reverse(NodePath.begin(),NodePath.end());
        //路径拼接，将可以直连的部分直接拼接。
        nav_msgs::Path beforepath;
        nav_msgs::Path lastpath;
        for(auto iter = NodePath.begin(); iter < NodePath.end(); ++iter){
            Eigen::Vector2d pt1;
            Eigen::Vector2d pt2;

            geometry_msgs::PoseStamped temppose1;
            geometry_msgs::PoseStamped temppose2;

            Index2Coord((*iter).index,temppose1);
            Index2Coord((NodePath.back()).index,temppose2);
            
            pt1[0] = temppose1.pose.position.x;
            pt1[1] = temppose1.pose.position.y;
            pt2[0] = temppose2.pose.position.x;
            pt2[1] = temppose2.pose.position.y;
            if(iter == NodePath.begin()){
                pt1[0] = points[i - 1].x;
                pt1[1] = points[i - 1].y;
                pt2[0] = points[i].x;
                pt2[1] = points[i].y;
            }
            // std::cout << "i = " << i<< std::endl;
            // std::cout << pt1 << std::endl;
            // std::cout << pt2 << std::endl;
            //全部直连路径都碰撞了，直接返回完整a*路径。
            auto next = iter + 1;
            if(next == NodePath.end()){
                ROS_INFO("a* path return!");
                for(int beforei = 0; beforei < beforepath.poses.size(); beforei++)
                    pathorder.poses.push_back(beforepath.poses[beforei]);
                break;
            }
            //将每个路径点传入pathorder
            lastpath.poses.clear();
            if(Oneshot(pt1, pt2,lastpath)){
                
                for(int beforei = 0; beforei < beforepath.poses.size(); beforei++)
                    pathorder.poses.push_back(beforepath.poses[beforei]);
                // ROS_INFO("Oneshot cuccess！ ");
                
                for(int lasti = 0; lasti < lastpath.poses.size(); lasti++)
                    pathorder.poses.push_back(lastpath.poses[lasti]);
                lastpath.poses.clear();
                break;
            }
            else{
                temppose1.header.frame_id = Cost_Map.header.frame_id;
                beforepath.poses.push_back(temppose1);
            }

        }
        for(int pti = 0; pti < pathorder.poses.size(); pti++)
            anspath.poses.push_back(pathorder.poses[pti]);
        //记录第一段路径
        if(i == 1){
            thefirstpath.poses.clear();
            firstpathindex = anspath.poses.size();
            thefirstpath = anspath;
        }
        
        NodePath.clear();
        pathorder.poses.clear();
        relieveNodemap(NodeMap);
    }

    path = anspath;
    return true;
}

bool path_search::replan(geometry_msgs::PoseStamped odompose, nav_msgs::Path &path, const nav_msgs::OccupancyGrid &map){

    nav_msgs::Path temppath;

    pointnode node1,node2;

    node1.x = odompose.pose.position.x;
    node1.y = odompose.pose.position.y;
    Eigen::Vector3d euler = Quataition2Euler(odompose.pose.orientation);
    node1.angle = euler[2];
    newreplanpoint[0].angle = euler[2];

    for(int i = 0; i < thefirstpath.poses.size() - 1; ++i){
        // std::cout << " i   = " <<  i  << std::endl;
        tydubins dubins; //dubins 曲线类，用于生成起点为 odompose 和路径为path连接的曲线。
        
        node2.x = thefirstpath.poses[i].pose.position.x;
        node2.y = thefirstpath.poses[i].pose.position.y;
        // std::cout << " node1.x  = " <<  node1.x  << std::endl;
        // std::cout << " node1.y  = " <<  node1.y  << std::endl;
        // std::cout << " node2.x  = " <<  node2.x  << std::endl;
        // std::cout << " node2.y  = " <<  node2.y  << std::endl;
        
        // if(node1.x == node2.x){
        //     continue;
        // }
        double dx = thefirstpath.poses[i + 1].pose.position.x - thefirstpath.poses[i].pose.position.x;
        double dy = thefirstpath.poses[i + 1].pose.position.y - thefirstpath.poses[i].pose.position.y;
        node2.angle = atan2(dy,dx);
        double resnum = (node1.x - node2.x)*(node1.x - node2.x) + (node1.y - node2.y)*(node1.y - node2.y);
        resnum = sqrt(resnum);

        // std::cout << "dx = " << dx << std::endl;
        // std::cout << "dy = " << dy << std::endl;

        // std::cout << "node1.angle = " << node1.angle << std::endl;
        // std::cout << "node2.angle = " << node2.angle << std::endl;

        // std::cout << "resnum = " << resnum << std::endl;
        if( resnum < 4.0*mMinRadius)
            continue;

        if (dubins.Easyconect(node1, node2, map))
        {   //在终点为i时候的路径点可以连接dubins曲线，处理并返回。
            temppath.poses.clear();
            temppath = dubins.GetdubinsPath(); 
            // PubPathCircle11 = dubins.PubPathCircle11;
            // PubPathCircle12 = dubins.PubPathCircle12;
            // PubPathCircle21 = dubins.PubPathCircle21;
            // PubPathCircle22 = dubins.PubPathCircle22;
            // Make_point1 = dubins.Make_point1;
            for(int jj = i + 1; jj < path.poses.size(); ++jj){
                temppath.poses.push_back(path.poses[jj]);
            }
            temppath.header = path.header;
            path = temppath;
            return true;      
        }
    }
    std::cout << "没有最小路径！！！" << std::endl;

    if(newreplan(newreplanpoint, path)){return true;}

    return false;
}

bool path_search::pointsinit(const waypoint &start,const waypoint &end, const nav_msgs::OccupancyGrid &map){
    //map init

    if(!init_Curve_Nodemap())
        return false;

    //startpose init
    Startpose.header = map.header;

    Startpose.pose.position.x = start.x;
    Startpose.pose.position.y = start.y;
    Startpose.pose.position.z = 0;
    Eigen::Vector3d euler = Quataition2Euler(Startpose.pose.orientation);
    mStartAngle = euler[2];

    // ROS_INFO("startpose 已经接收 ");    
    Coord2Index(Start_index, Startpose);
    if(OutOfrange(Start_index)){
        ROS_ERROR("连接点起点 在地图外！ ");
        return false;
    }

    //endpose init
    Endpose.header = map.header;
    Endpose.pose.position.x = end.x;
    Endpose.pose.position.y = end.y;
    Endpose.pose.position.z = 0;

    // Eigen::Vector3d euler = Quataition2Euler(msg.pose.orientation);
    // mEndAngle = euler[2];

    // ROS_INFO("endpose 已经接收 ");
    // std::cout << "x =  " << Endpose.pose.position.x << std::endl;
    // std::cout << "y =  " << Endpose.pose.position.y << std::endl;
    // std::cout << "z =  " << Endpose.pose.position.z << std::endl;
    Coord2Index(End_index, Endpose);
    if(OutOfrange(End_index)){
        ROS_ERROR("连接点终点 在地图外！ ");
        return false;
    }

    return true;
}

bool path_search::run()
{
    GridNodePtr end_nodeptr;
    ros::Time t1 = ros::Time::now();
    if(OutOfrange(Start_index)){
        ROS_ERROR("连接点起点位于地图外！！ !");
        return false;
    }
    else{
        current_nodeptr = NodeMap[Start_index[0]][Start_index[1]];
        end_nodeptr = NodeMap[End_index[0]][End_index[1]];
    }

    current_nodeptr->state = GridNode::OPENSET;
    current_nodeptr->gScore = 0;
    current_nodeptr->hScore =  weight*Cul_distance(current_index, End_index);
    current_nodeptr->cameFrom = NULL;
    current_nodeptr->angle = mStartAngle;

    if(!CheckCurrentPose(current_nodeptr)){
        ROS_ERROR("起点附近不能通行 !");
        return false;
    };

    if(!CheckCurrentPose(end_nodeptr)){
        std::cout << "终点值为" <<end_nodeptr->index<<std::endl;
        ROS_ERROR("终点附近不能通行 !");
        return false;
    };

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet.swap(empty);
    
    openSet.push(current_nodeptr);
    while (!openSet.empty())
    {
        if (current_nodeptr->index == End_index)
        {
            // ROS_INFO("路径搜索完毕 请获取path !");
            break;
        }
        current_nodeptr->state = GridNode::CLOSEDSET;
        GridNodePtr tempPtr;
        for (int i = -1; i <=1; i++)
        {
            for (int j = -1; j<=1; j++)
            {
                if (i == 0 && j == 0)
                {
                    continue;
                }
                Eigen::Vector2i checkindex;
                checkindex[0] = current_nodeptr->index[0] + i;
                checkindex[1] = current_nodeptr->index[1] + j;
                if(OutOfrange(checkindex)){continue;}
                tempPtr = NodeMap[current_nodeptr->index[0] + i][current_nodeptr->index[1] + j];

                if (Is_obstacle(tempPtr)){continue;}
                if(tempPtr->state == GridNode::CLOSEDSET){continue;}

                if (tempPtr->state == GridNode::OPENSET)
                {
                    double temp_sum_1 = tempPtr->gScore ;
                    double temp_sum_2 = current_nodeptr->gScore + Cul_distance(current_nodeptr->index,tempPtr->index);

                    if (temp_sum_1 < temp_sum_2)
                    {
                        continue;
                    }
                    else{
                        tempPtr->cameFrom = current_nodeptr;
                        tempPtr->gScore = current_nodeptr->gScore + Cul_distance(current_nodeptr->index, tempPtr->index);
                        tempPtr->hScore = tempPtr->gScore + weight*GetHeu(tempPtr->index);
                    }
                }
                if(tempPtr->state == GridNode::UNDEFINED){
                    tempPtr->state = GridNode::OPENSET;
                    tempPtr->cameFrom = current_nodeptr;
                    tempPtr->gScore = current_nodeptr->gScore + Cul_distance(current_nodeptr->index, tempPtr->index);
                    tempPtr->hScore = tempPtr->gScore +  weight*GetHeu(tempPtr->index);
                    openSet.push(tempPtr);
                }

            }
        }
        current_nodeptr = openSet.top();
        openSet.pop();
    }
    // ros::Time t2 = ros::Time::now();
    // ROS_INFO("a*****时间为 %.8f s",t2.toSec()-t1.toSec());
    return true;
}

bool path_search::Oneshot(Eigen::Vector2d pt1,Eigen::Vector2d pt2, nav_msgs::Path &partpath){
    nav_msgs::Path anspath;
    anspath.poses.clear();
    double dx = pt2[0] - pt1[0];
    double dy = pt2[1] - pt1[1];

    double angle = atan2(dy,dx);
    double length = sqrt(dx*dx + dy*dy);
    // std::cout << "pt1 " << pt1<< std::endl; 
    // std::cout << "pt2"  << pt2 <<std::endl; 

    bool havecollition = 0;
    for(double ilength = 0; ilength < length; ilength += 4.5){
        double tempx = pt1[0] + ilength * cos(angle);
        double tempy = pt1[1] + ilength * sin(angle);

        Eigen::Vector2i tempindex;
        tempindex[0] = int((tempx - Cost_Map.info.origin.position.x) / resolution);
        tempindex[1] = int((tempy - Cost_Map.info.origin.position.x) / resolution);
        if(OutOfrange(tempindex) || Is_obstacle(NodeMap[tempindex[0]][tempindex[1]])){
            // std::cout << "pt1 " << pt1<< std::endl; 
            // std::cout << "pt2"  << pt2 <<std::endl; 
            // std::cout << "tempindex"  << tempindex <<std::endl; 
            havecollition = 1; 
            break;
        }
        else{
            geometry_msgs::PoseStamped temppose;
            temppose.header.frame_id = Cost_Map.header.frame_id;
            temppose.pose.position.x = tempx;
            temppose.pose.position.y = tempy;
            temppose.pose.position.z = 0;
            anspath.poses.push_back(temppose);
            // std::cout << temppose.pose.position <<std::endl; 
        }
    }

    if(!havecollition){
        // std::cout << "no collition!!"  <<std::endl; 
        partpath = anspath;
        return true;
    }
    else
        return false;
    
}

bool path_search::newreplan(const std::vector<waypoint> &points, nav_msgs::Path &path){
    tydubins dubins;
    nav_msgs::Path dubinspath;
    nav_msgs::Path Oneshotpath;
    //计算假设相对于第一段路径的末端的相对角度。
    double dx = points[0].x - points[1].x;
    double dy = points[0].y - points[1].y;
    double angle1 = remainder(atan2(dy,dx), 2.0*M_PI);

    dx = path.poses[firstpathindex + 1].pose.position.x - path.poses[firstpathindex].pose.position.x;
    dy = path.poses[firstpathindex + 1].pose.position.y - path.poses[firstpathindex].pose.position.y;
    double angle2 = remainder(atan2(dy,dx), 2.0*M_PI);
    double angle = -angle2;
    if(angle2 - angle1 > 0 && angle2 - angle1 < M_PI/2.0 + M_PI/6.0)
        angle = angle2 + M_PI/2.0; 
    if(angle2 - angle1 < 0 && angle2 - angle1 > -1.0*M_PI/2.0 - M_PI/6.0)
        angle = angle2 - M_PI/2.0;
    
    //获取第一段路径到增加点的dubins曲线 
    pointnode newnode1;
    pointnode newnode2;
    newnode1.x = points[0].x ;
    newnode1.y = points[0].y ;
    newnode1.angle = points[0].angle;

    newnode2.x = points[1].x + 4.0*mMinRadius*cos(angle);
    newnode2.y = points[1].y + 4.0*mMinRadius*sin(angle);
    newnode2.angle = remainder(angle + M_PI, 2.0*M_PI);
    // std::cout <<"newnode2.angle = " << newnode2.angle  << std::endl; 
    if(dubins.Easyconect(newnode1,newnode2,Cost_Map))
        dubinspath = dubins.GetdubinsPath();
    else{
        ROS_INFO("GetdubinsPath 失败 ！");   
        return false;

    }
    //获取增加点到第一段路径末端的a*路径。
    std::vector<waypoint> Oneshotpoints;
    waypoint point;
    point.x = newnode2.x;
    point.y = newnode2.y;
    Oneshotpoints.push_back(point);
    Oneshotpoints.push_back(points[1]);
    //将前半段路径拼接并将原来的曲线保留
    astarOneshot(Oneshotpoints, Oneshotpath);
    // ROS_INFO("astarOneshot 成功 ！");
    for(int i = 0; i < Oneshotpath.poses.size(); ++i)
        dubinspath.poses.push_back(Oneshotpath.poses[i]);
    for(int i = firstpathindex; i < path.poses.size(); ++i)
        dubinspath.poses.push_back(path.poses[i]);
    
    path.poses = dubinspath.poses;
    Oneshotpath.poses.clear();

    return true;

}

bool path_search::init_Curve_Nodemap()
{

    //map init
    map_leng_x = Cost_Map.info.width;
    map_leng_y = Cost_Map.info.height;

    resolution = Cost_Map.info.resolution;
    map_index_x = Cost_Map.info.width;
    map_index_y = Cost_Map.info.height;
    // ROS_INFO("Costmap 已经接收 ");
    
    if(map_leng_x == 0){
        ROS_ERROR("地图不存在！ ");
        return false;
    }

    for (int i = 0; i < 800; i++)
        for (int j = 0; j < 800; j++)
        {
            Set_obstacle(NodeMap[i][j]);
        }
    // ROS_INFO("Nodemap 已经初始化 ");
    return true;
}

void path_search::relieveNodemap(GridNodePtr **NodeMap){

    for (int i = 0; i < map_index_x; i++)
    {
        for (int j = 0; j < map_index_y; j++)
        {
            NodeMap[i][j]->index[0] = i;
            NodeMap[i][j]->index[1] = j;
            NodeMap[i][j]->state = GridNode::UNDEFINED;
            NodeMap[i][j]->angle = 0;
            NodeMap[i][j]->gScore = inf;
            NodeMap[i][j]->hScore = inf;
            NodeMap[i][j]->cameFrom = 0;
        }
    }
    // ROS_INFO("Nodemap 已经释放完毕 ");

}   


void path_search::OccumapCB(const nav_msgs::OccupancyGrid &msg){
    Cost_Map = msg;
    map_leng_x = msg.info.width;
    map_leng_y = msg.info.height;

    resolution = msg.info.resolution;
    map_index_x = msg.info.width;
    map_index_y = msg.info.height;

    // ROS_INFO("Costmap 已经接收 ");

    occumap_flag = true;
}


void path_search::StartposeCB(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    Startpose.header = msg.header;
    Startpose.pose = msg.pose.pose;
    
    Eigen::Vector3d euler = Quataition2Euler(msg.pose.pose.orientation);
    mStartAngle = euler[2];

    ROS_INFO("startpose 已经接收 ");
    std::cout << "x =  " << Startpose.pose.position.x << std::endl;
    std::cout << "y =  " << Startpose.pose.position.y << std::endl;
    std::cout << "z =  " << Startpose.pose.position.z << std::endl;
    Coord2Index(Start_index, Startpose);

    start_flag = true;
}

void path_search::EndposeCB(const geometry_msgs::PoseStamped &msg)
{
    Endpose = msg;
    Eigen::Vector3d euler = Quataition2Euler(msg.pose.orientation);
    mEndAngle = euler[2];

    ROS_INFO("endpose 已经接收 ");
    std::cout << "x =  " << Endpose.pose.position.x << std::endl;
    std::cout << "y =  " << Endpose.pose.position.y << std::endl;
    std::cout << "z =  " << Endpose.pose.position.z << std::endl;
    Coord2Index(End_index, Endpose);

    end_flag = true;
}

double path_search::GetHeu(Eigen::Vector2i index)
{
    double Sum_Heu;

    double h1 = Cul_distance(index, End_index);
    double h2 = abs(End_index[0] - index[0]) + abs(End_index[1] - index[1]);
    double h3 = 0;

    Sum_Heu = 2.0*h2;
    return Sum_Heu;
}

bool path_search::CheckCurrentPose(GridNodePtr nodeptr){

    if(Is_obstacle(nodeptr)){
        ROS_ERROR("连接点附近有障碍物～～！！ ");
        return false;
    }

    return true;
}

bool path_search::OutOfrange(const Eigen::Vector2i index){
    return (index[0] < 0 || index[0] >= map_index_x ||index[1] < 0 ||index[1] >= map_index_y);
}

void path_search::Set_obstacle(GridNodePtr nodeptr)
{
    int index;
    int cost;

    index = nodeptr->index[1] * Cost_Map.info.width + nodeptr->index[0];
    cost = Cost_Map.data[index];
    if (0 != cost)
    {
        nodeptr->state = GridNode::OBSTACLE;
    }
    // std::cout << "cost = " << cost << std::endl;
}

bool path_search::Getpath()
{
    NodePath.clear();
    PubPath.poses.clear();
    PubPath.header = Cost_Map.header;
    NodePath.push_back(*current_nodeptr);
    while (current_nodeptr->index != Start_index)
    {
        current_nodeptr = current_nodeptr->cameFrom;
        NodePath.push_back(*current_nodeptr);
    }
    reverse(NodePath.begin(),NodePath.end());

    for(auto iter : NodePath){
        geometry_msgs::PoseStamped temppose;
        Index2Coord(iter.index,temppose);
        temppose.header.frame_id = Cost_Map.header.frame_id;
        PubPath.poses.push_back(temppose);
    }
    PubPath.header.frame_id = Cost_Map.header.frame_id;
    path_flag = true;
    return true;
}

nav_msgs::Path path_search::bspline_path(const nav_msgs::Path &rawpath){
    nav_msgs::Path respath;
    nav_msgs::Path rawpathss = rawpath;

    Eigen::MatrixXd opt_points;
    Eigen::VectorXd point(2);
    tybspline mbspline;
    std::cout<< "rawpath.size（） = "<< rawpath.poses.size() <<  std::endl;
    int length = rawpath.poses.size();
    if(length < 16)
        return rawpathss;
    
    for(int i = 0; i < length; i += 4){
        point[0] = rawpath.poses[i].pose.position.x;
        point[1] = rawpath.poses[i].pose.position.y;
        if(i == (length -1)){
            // std::cout << "point[0]  = " << point[0] << std::endl;
            // std::cout << "point[1]  = " << point[1] << std::endl;
        }
        if(i == 0){
            // std::cout << "000point[0]  = " << point[0] << std::endl;
            // std::cout << "111point[1]  = " << point[1] << std::endl;
        }

        opt_points.conservativeResize(2, opt_points.cols() + 1);
        opt_points.col(opt_points.cols() - 1) = point;       
    }
    mbspline.init(opt_points,3);
    mbspline.setUniformKnot();

    respath.header = rawpath.header;
    double denselength = 5.0*length;
    for(double i = 0; i <= denselength; ++i){
        double u = i / denselength;
        // std::cout << "uuu = = " << u << std::endl;
    
        Eigen::VectorXd temppoint = mbspline.evaluateDeBoor(u);
        
        geometry_msgs::PoseStamped temppose;
        temppose.pose.position.x = temppoint[0];
        temppose.pose.position.y = temppoint[1];
        // std::cout << "temppose.pose.position.x  = " << temppose.pose.position.x << std::endl;
        // std::cout << "temppose.pose.position.y  = " << temppose.pose.position.y << std::endl;
        respath.poses.push_back(temppose);
    }
    
    // respath.poses.push_back(rawpath.poses[rawpath.poses.size() - 1]);
    // std::cout << "respath.poses[0].pose.position.x = " <<  respath.poses[0].pose.position.x << std::endl;
    // std::cout << "respath.poses[0].pose.position.y = " <<  respath.poses[0].pose.position.y << std::endl;

    // std::cout << "respath.poses[respath.poses.size() - 1].pose.position.x = " <<  respath.poses[respath.poses.size() - 1].pose.position.x << std::endl;
    // std::cout << "respath.poses[respath.poses.size() - 1].pose.position.y = " <<  respath.poses[respath.poses.size() - 1].pose.position.y << std::endl;
    return respath;
}

