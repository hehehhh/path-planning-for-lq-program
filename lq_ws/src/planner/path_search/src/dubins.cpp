#include <path_search/dubins.h>

tydubins::tydubins(std::vector<int> inputstart, std::vector<int> inputend, nav_msgs::OccupancyGrid map){
    start.x = inputstart[0];
    start.y = inputstart[1];
    start.angle = inputstart[2];
    
    end.x = inputstart[0];
    end.y = inputstart[1];
    end.angle = inputstart[2];

    MAP = map;
}

tydubins::~tydubins(){}
tydubins::tydubins(){}

bool tydubins::Easyconect(pointnode start, pointnode end, nav_msgs::OccupancyGrid map){
    resolution = map.info.resolution;
    map_index_x = map.info.width;
    map_index_y = map.info.height;
    MAP = map;

    Eigen::Vector2d position1;
    Eigen::Vector2d position2;
    bool left = 1;
    bool right = 0;
    //初始化检测起点和终点
    position1[0] = start.x;
    position1[1] = start.y;
    position2[0] = end.x;
    position2[1] = end.y;
    // std::cout << "position1 =  " << position1 << std::endl;
    // std::cout << "position2 =  " << position2 << std::endl;
    centre11 = GetCentreCircle(start,left); 
    centre12 = GetCentreCircle(start,right); 
    centre21 = GetCentreCircle(end,left); 
    centre22 = GetCentreCircle(end,right);

    // Getpathcircle(centre11,centre12,centre21,centre22);

    GetPointOfContact(centre11,centre21,Cpoint1,Cpoint2,LSL);//左左圆 LSL 外切圆
    GetPointOfContact(centre11,centre22,Cpoint3,Cpoint4,LSR);//左右圆 LSR 内切圆
    GetPointOfContact(centre12,centre21,Cpoint5,Cpoint6,RSL);//右左圆 RSL 内切圆
    GetPointOfContact(centre12,centre22,Cpoint7,Cpoint8,RSR);//右右圆 RSR 外切圆
    // GetPointOfContactPub(Cpoint1,Cpoint2,Cpoint3,Cpoint4,Cpoint5,Cpoint6,Cpoint7,Cpoint8);

    // std::cout << "centre11 =  " << centre11 << std::endl;
    // std::cout << "centre12 =  " << centre12 << std::endl;
    // std::cout << "centre21 =  " << centre21 << std::endl;
    // std::cout << "centre22 =  " << centre22 << std::endl;
    // std::cout << "point1 =  " << point1 << std::endl;
    // std::cout << "point2 =  " << point2 << std::endl;
    // std::cout << "cccccpoint3 =  " << Cpoint3 << std::endl;
    // std::cout << "cccccpoint4 =  " << Cpoint4 << std::endl;
    // std::cout << "point5 =  " << point5 << std::endl;
    // std::cout << "point6 =  " << point6 << std::endl;
    // std::cout << "point7 =  " << point7 << std::endl;
    // std::cout << "point8 =  " << point8 << std::endl;
    

    lengths[0] = Cul_S(position1,position2,centre11,centre21,Cpoint1,Cpoint2,LSL,lineListS);
    lengths[1] = Cul_S(position1,position2,centre11,centre22,Cpoint3,Cpoint4,LSR,lineListS);
    lengths[2] = Cul_S(position1,position2,centre12,centre21,Cpoint5,Cpoint6,RSL,lineListS);
    lengths[3] = Cul_S(position1,position2,centre12,centre22,Cpoint7,Cpoint8,RSR,lineListS);
    if(!lineListS.empty()){
        mDubinsPath.poses.clear();
        auto iter = lineListS.begin();
        if(iter->first == inf){
            // std::cout << "没有最小路径！！！" << iter->first << std::endl;
            return false;
        }
        mDubinsPath = mtempPathList[iter->second];
        // std::cout << "路径的最小长度 =  " << iter->first << std::endl;
        // std::cout << "路径的类型为 ：" << iter->second << std::endl;
        // ROS_INFO("我在这儿成了 ");
        return true;
    }
    return false;
}


void tydubins::GetPointOfContact(const Eigen::Vector2d centre1, const Eigen::Vector2d centre2,
Eigen::Vector2d &point1 ,Eigen::Vector2d &point2, int type, Eigen::Vector2d &Circle){
    if(type == 1 || type == 2){
        // 右圆对应右转对应逆时针，内切圆对应左右圆或右左圆，外切圆对应左左圆或右右圆
        // 逆时针对应切点相对于两圆心连线顺时针
        // 此处为内切圆计算
        double flag = 1.0;
        if(type == 1){
            flag = -1.0;//左右圆对应为相对为减，右左圆对应为加，画图可知
        }
        double dx = centre1[0] - centre2[0];
        double dy = centre1[1] - centre2[1];
        double length = sqrt(dx*dx + dy*dy)/2.0;

        double temp_angleOfcentre1 = acos(mMinra/length);
        double temp_centreRelativeXaxis1 = atan2((centre2[1] - centre1[1]),(centre2[0] - centre1[0]));

        double angle1 = temp_centreRelativeXaxis1 + flag*temp_angleOfcentre1;
        point1[0] = centre1[0] + mMinra * cos(angle1);
        point1[1] = centre1[1] + mMinra * sin(angle1);

        double temp_angleOfcentre2 = acos(mMinra/length);
        double temp_centreRelativeXaxis2 = atan2((centre1[1] - centre2[1]),(centre1[0] - centre2[0]));

        double angle2 = temp_centreRelativeXaxis2 + flag*temp_angleOfcentre2;
        point2[0] = centre2[0] + mMinra * cos(angle2);
        point2[1] = centre2[1] + mMinra * sin(angle2);
        // std::cout << "GetPointOfContactpoint1 =  " << point1 << std::endl;
        // std::cout << "GetPointOfContactpoint2 =  " << point2 << std::endl;
    }
    if(type == 0 || type == 3){
        // 此处为外切圆计算
        double flag = 1.0; 
        if(type == 0){
            flag = -1.0; //右右圆对应为切点1加，左左圆对应切点2为减 ---- 右右圆对应为切点1减，左左圆对应切点2为加
        }
        double temp_centreRelativeXaxis1 = atan2((centre2[1] - centre1[1]),(centre2[0] - centre1[0]));

        double angle1 = temp_centreRelativeXaxis1 + flag*M_PI/2.0;
        point1[0] = centre1[0] + mMinra * cos(angle1);
        point1[1] = centre1[1] + mMinra * sin(angle1);

        double temp_centreRelativeXaxis2 = atan2((centre1[1] - centre2[1]),(centre1[0] - centre2[0]));

        double angle2 = temp_centreRelativeXaxis2 - flag*M_PI/2.0;
        point2[0] = centre2[0] + mMinra * cos(angle2);
        point2[1] = centre2[1] + mMinra * sin(angle2);
    }
    if(type == 4 || type ==5){
        double dx = centre2[0] - centre2[0];
        double dy = centre1[1] - centre1[1];
        double L1 =sqrt(dx*dx + dy*dy);

        double L2, L3 = 2.0 * mMinra;

        double angle2 = acos(L1/2.0/L2);

        double angle1to2;
        double absangle;

        if (centre1[1] == centre2[1]){
            angle1to2 = M_PI/2.0;
        }
        else{
            angle1to2 = atan2(centre2[1] - centre1[1], centre2[0] - centre1[0]);
        }

        if(type == 4){
            absangle = angle1to2 - angle2;
        }
        else if (type == 5){
            absangle = angle1to2 + angle2;
        }

        Circle[0] = centre1[0] + L2 * cos(absangle);
        Circle[1] = centre1[1] + L2 * sin(absangle);

        point1[0] = (centre1[0] + Circle[0])/2; 
        point1[1] = (centre1[1] + Circle[1])/2; 

        point2[0] = (centre2[0] + Circle[0])/2; 
        point2[1] = (centre2[1] + Circle[1])/2; 
    }
    return;
}

void tydubins::InsertPubPointEA(const Eigen::Vector2d coord){
    geometry_msgs::PoseStamped temppose;
    temppose.pose.position.x = coord[0] ;
    temppose.pose.position.y = coord[1] ;
    // std::cout << "temppose = " << temppose.pose.position << std::endl;
    
    mDubinsPath.poses.push_back(temppose);
}

double tydubins::Cul_S(const Eigen::Vector2d position1, const Eigen::Vector2d position2, 
const Eigen::Vector2d centre1, const Eigen::Vector2d centre2, 
const Eigen::Vector2d point1, const Eigen::Vector2d point2, 
const int type,std::map<double, int> &lineListS){
    double l1,l2,l3;
    double S = inf;
    double theta1 = remainder(atan2(position1[1] - centre1[1],position1[0] - centre1[0]), 2.0*M_PI);
    double theta2 = remainder(atan2(point1[1] - centre1[1],point1[0] - centre1[0]), 2.0*M_PI);

    double theta3 = remainder(atan2(point2[1] - centre2[1],point2[0] - centre2[0]), 2.0*M_PI);
    double theta4 = remainder(atan2(position2[1] - centre2[1],position2[0] - centre2[0]), 2.0*M_PI);

    double symbol1;
    double symbol2;

    double alpha1;
    double alpha2;

    if(theta2 >= theta1){
        alpha1 = theta2 - theta1;
    }
    else{
        alpha1 = theta1 - theta2;
    }

    if(theta4 >= theta3){
        alpha2 = theta4 - theta3;
    }else{
        alpha2 = theta3 - theta4;
    }

    switch (type)
    {
    case 0:
        symbol1 = 1.0;
        symbol2 = 1.0;

        if(theta2 >= theta1){
            alpha1 = alpha1;
        }
        else{
            alpha1 = 2.0*M_PI - alpha1;
        }
        
        if(theta4 >= theta3){
            alpha2 = alpha2;
        }else{
            alpha2 = 2.0*M_PI - alpha2;
        }
        break;
    case 1:
        symbol1 = 1.0;
        symbol2 = -1.0;

        if(theta2 >= theta1){
            alpha1 = alpha1;
        }
        else{
            alpha1 = 2.0*M_PI - alpha1;
        }
        
        if(theta4 >= theta3){
            alpha2 = 2.0*M_PI - alpha2;
        }else{
            alpha2 = alpha2;
        }
        break;
    case 2:
        symbol1 = -1.0;
        symbol2 = 1.0;

        if(theta2 >= theta1){
            alpha1 = 2.0*M_PI - alpha1 ;
        }
        else{
            alpha1 = alpha1;
        }
        
        if(theta4 >= theta3){
            alpha2 = alpha2;
        }else{
            alpha2 = 2.0*M_PI - alpha2;
        }
        break;
    case 3:
        symbol1 = -1.0;
        symbol2 = -1.0;

        if(theta2 >= theta1){
            alpha1 = 2.0*M_PI - alpha1;
        }
        else{
            alpha1 = alpha1;
        }
        
        if(theta4 >= theta3){
            alpha2 = 2.0*M_PI - alpha2;
        }else{
            alpha2 = alpha2;
        }
        
        break;
    default:
        break;
    }

    l1 = alpha1 * mMinra;
    double dx = point2[0] - point1[0];
    double dy = point2[1] - point1[1];
    l2 = sqrt(dx*dx + dy*dy);

    l3 = alpha2 * mMinra;

    S = l1 + l2 + l3;

    bool havecollition = 0;
    Eigen::Vector2d tempoint;
    mDubinsPath.poses.clear();
    InsertPubPointEA(position1);

    double startangle1 = theta1;
    for (double iangle = 0.0; iangle < alpha1; iangle += 0.02){
        tempoint[0] = centre1[0] + mMinra*cos(startangle1 + symbol1*iangle);
        tempoint[1] = centre1[1] + mMinra*sin(startangle1 + symbol1*iangle);

        if(Is_obstacle(tempoint)){

            havecollition = 1;  
            break;
        }

        InsertPubPointEA(tempoint);
    }
    // std::cout << "111tempoint = " << tempoint << std::endl;
    // std::cout << "111point1 = " << point1 << std::endl;
    if(!havecollition){
        double tempgradient;
        tempgradient = atan2(point2[1] - point1[1], point2[0] - point1[0]);
        for (double ilength = 0.0; ilength < l2; ilength += 0.2){
            tempoint[0] = point1[0] + ilength*cos(tempgradient);
            tempoint[1] = point1[1] + ilength*sin(tempgradient);

            if(Is_obstacle(tempoint)){
                havecollition = 1;  
                break;
            }
            InsertPubPointEA(tempoint);
        }
    }
    // std::cout << "222tempoint = " << tempoint << std::endl;
    // std::cout << "222point2 = " << point2 << std::endl;
    if(!havecollition){
        double startangle3 = theta3; 
        for (double iangel = 0; iangel < alpha2; iangel += 0.02){
            tempoint[0] = centre2[0] + mMinra*cos(startangle3 + symbol2*iangel);
            tempoint[1] = centre2[1] + mMinra*sin(startangle3 + symbol2*iangel);

            if(Is_obstacle(tempoint)){
                havecollition = 1;  
                break;
            }
            InsertPubPointEA(tempoint);
        }
    }

    if(havecollition){
        // std::cout << "mDubinsPath.poses[(mDubinsPath.poses.size() - 1)] = " << mDubinsPath.poses[(mDubinsPath.poses.size() - 1)] <<  std::endl;
        mDubinsPath.poses.clear();
        // std::cout << "撞障碍物了 ！" << std::endl;
        lineListS.insert({inf,type});
        return inf;
    }
    else{
        mtempPathList[type] = mDubinsPath;
        lineListS.insert({S,type});
        // std::cout << "alpha1 =  " << alpha1 <<std::endl;
        // std::cout << "alpha2 =  " << alpha2 <<std::endl;

        // std::cout << "theta1 =  " << theta1 <<std::endl;
        // std::cout << "theta2 =  " << theta2 <<std::endl;
        // std::cout << "theta3 =  " << theta3 <<std::endl;
        // std::cout << "theta4 =  " << theta4 <<std::endl;

        // std::cout << "type =  " << type << "好了！！"<<std::endl;
        // std::cout << "type的 SS =  " << S << std::endl;
        return S;
    }

}

nav_msgs::Path tydubins::GetdubinsPath(){
    return mDubinsPath;
}

void tydubins::Getpathcircle(const Eigen::Vector2d circle11, const Eigen::Vector2d circle12, const Eigen::Vector2d circle21, const Eigen::Vector2d circle22){
    PubPathCircle11.poses.clear();
    PubPathCircle12.poses.clear();
    PubPathCircle21.poses.clear();
    PubPathCircle22.poses.clear();
    PubPathCircle11.header.frame_id = MAP.header.frame_id;
    PubPathCircle12.header.frame_id = MAP.header.frame_id;
    PubPathCircle21.header.frame_id = MAP.header.frame_id;
    PubPathCircle22.header.frame_id = MAP.header.frame_id;
    
    for (double iangle = 0; iangle <= 2*M_PI; iangle += 0.01){
        geometry_msgs::PoseStamped temppose11;
        geometry_msgs::PoseStamped temppose12;
        geometry_msgs::PoseStamped temppose21;
        geometry_msgs::PoseStamped temppose22;

        temppose11.header.frame_id = MAP.header.frame_id;
        temppose12.header.frame_id = MAP.header.frame_id;
        temppose21.header.frame_id = MAP.header.frame_id;
        temppose22.header.frame_id = MAP.header.frame_id;

        temppose11.pose.position.x = circle11[0] + mMinra * cos(iangle); 
        temppose11.pose.position.y = circle11[1] + mMinra * sin(iangle);

        temppose12.pose.position.x = circle12[0] + mMinra * cos(iangle); 
        temppose12.pose.position.y = circle12[1] + mMinra * sin(iangle); 

        temppose21.pose.position.x = circle21[0] + mMinra * cos(iangle); 
        temppose21.pose.position.y = circle21[1] + mMinra * sin(iangle); 

        temppose22.pose.position.x = circle22[0] + mMinra * cos(iangle); 
        temppose22.pose.position.y = circle22[1] + mMinra * sin(iangle); 

        PubPathCircle11.poses.push_back(temppose11);
        PubPathCircle12.poses.push_back(temppose12);
        PubPathCircle21.poses.push_back(temppose21);
        PubPathCircle22.poses.push_back(temppose22);
    }
}

bool tydubins::PureDubins(pointnode start, pointnode end){

    Eigen::Vector2d position1;
    Eigen::Vector2d position2;
    bool left = 1;
    bool right = 0;
    //初始化检测起点和终点
    position1[0] = start.x;
    position1[1] = start.y;
    position2[0] = end.x;
    position2[1] = end.y;
    std::cout << "position1 =  " << position1 << std::endl;
    std::cout << "position2 =  " << position2 << std::endl;
    centre11 = GetCentreCircle(start,left); 
    centre12 = GetCentreCircle(start,right); 
    centre21 = GetCentreCircle(end,left); 
    centre22 = GetCentreCircle(end,right);

    GetPointOfContact(centre11,centre21,Cpoint1,Cpoint2,LSL);//左左圆 LSL 外切圆
    GetPointOfContact(centre11,centre22,Cpoint3,Cpoint4,LSR);//左右圆 LSR 内切圆
    GetPointOfContact(centre12,centre21,Cpoint5,Cpoint6,RSL);//右左圆 RSL 内切圆
    GetPointOfContact(centre12,centre22,Cpoint7,Cpoint8,RSR);//右右圆 RSR 外切圆

    // Getpathcircle(centre11,centre12,centre21,centre22);

    lengths[0] = PureDubinsCul_S(position1,position2,centre11,centre21,Cpoint1,Cpoint2,LSL,lineListS);
    lengths[1] = PureDubinsCul_S(position1,position2,centre11,centre22,Cpoint3,Cpoint4,LSR,lineListS);
    lengths[2] = PureDubinsCul_S(position1,position2,centre12,centre21,Cpoint5,Cpoint6,RSL,lineListS);
    lengths[3] = PureDubinsCul_S(position1,position2,centre12,centre22,Cpoint7,Cpoint8,RSR,lineListS);
    if(!lineListS.empty()){
        mDubinsPath.poses.clear();
        auto iter = lineListS.begin();
        if(iter->first == inf){
            std::cout << "没有最小路径！！！" << iter->first << std::endl;
            return false;
        }
        mDubinsPath = mtempPathList[iter->second];
        std::cout << "路径的最小长度 =  " << iter->first << std::endl;
        std::cout << "路径的类型为 ：" << iter->second << std::endl;
        ROS_INFO("我在这儿成了 ");
        return true;
    }
    return false;
}

double tydubins::PureDubinsCul_S(const Eigen::Vector2d position1, const Eigen::Vector2d position2, 
const Eigen::Vector2d centre1, const Eigen::Vector2d centre2, 
const Eigen::Vector2d point1, const Eigen::Vector2d point2, 
const int type,std::map<double, int> &lineListS){
    double l1,l2,l3;
    double S = inf;
    double theta1 = remainder(atan2(position1[1] - centre1[1],position1[0] - centre1[0]), 2*M_PI);
    double theta2 = remainder(atan2(point1[1] - centre1[1],point1[0] - centre1[0]), 2*M_PI);

    double theta3 = remainder(atan2(point2[1] - centre2[1],point2[0] - centre2[0]), 2*M_PI);
    double theta4 = remainder(atan2(position2[1] - centre2[1],position2[0] - centre2[0]), 2*M_PI);

    double symbol1;
    double symbol2;

    double alpha1;
    double alpha2;

    if(theta2 >= theta1){
        alpha1 = theta2 - theta1;
    }
    else{
        alpha1 = theta1 - theta2;
    }

    if(theta4 >= theta3){
        alpha2 = theta4 - theta3;
    }else{
        alpha2 = theta3 - theta4;
    }

    switch (type)
    {
    case 0:
        symbol1 = 1.0;
        symbol2 = 1.0;

        if(theta2 >= theta1){
            alpha1 = alpha1;
        }
        else{
            alpha1 = 2.0*M_PI - alpha1;
        }
        
        if(theta4 >= theta3){
            alpha2 = alpha2;
        }else{
            alpha2 = 2.0*M_PI - alpha2;
        }
        break;
    case 1:
        symbol1 = 1.0;
        symbol2 = -1.0;

        if(theta2 >= theta1){
            alpha1 = alpha1;
        }
        else{
            alpha1 = 2.0*M_PI - alpha1;
        }
        
        if(theta4 >= theta3){
            alpha2 = 2.0*M_PI - alpha2;
        }else{
            alpha2 = alpha2;
        }
        break;
    case 2:
        symbol1 = -1.0;
        symbol2 = 1.0;

        if(theta2 >= theta1){
            alpha1 = 2.0*M_PI - alpha1 ;
        }
        else{
            alpha1 = alpha1;
        }
        
        if(theta4 >= theta3){
            alpha2 = alpha2;
        }else{
            alpha2 = 2.0*M_PI - alpha2;
        }
        break;
    case 3:
        symbol1 = -1.0;
        symbol2 = -1.0;

        if(theta2 >= theta1){
            alpha1 = 2.0*M_PI - alpha1;
        }
        else{
            alpha1 = alpha1;
        }
        
        if(theta4 >= theta3){
            alpha2 = 2.0*M_PI - alpha2;
        }else{
            alpha2 = alpha2;
        }
        
        break;
    default:
        break;
    }

    l1 = alpha1 * mMinra;
    double dx = point2[0] - point1[0];
    double dy = point2[1] - point1[1];
    l2 = sqrt(dx*dx + dy*dy);

    l3 = alpha2 * mMinra;

    S = l1 + l2 + l3;

    bool havecollition = 0;
    Eigen::Vector2d tempoint;
    mDubinsPath.poses.clear();
    InsertPubPointEA(position1);

    double startangle1 = theta1;
    for (double iangle = 0; iangle < alpha1; iangle += 0.1){
        tempoint[0] = centre1[0] + mMinra*cos(startangle1 + symbol1*iangle);
        tempoint[1] = centre1[1] + mMinra*sin(startangle1 + symbol1*iangle);
        // std::cout << "centre1[0] = " << centre1[0] << std::endl;
        // std::cout << "centre1[1] = " << centre1[1] << std::endl;

        // std::cout << "tempoint[0] = " << tempoint[0] << std::endl;
        // std::cout << "tempoint[1] = " << tempoint[1] << std::endl;

        // std::cout << "mMinra*cos(startangle1 + symbol1*iangle) = " << mMinra*cos(startangle1 + symbol1*iangle)<< std::endl;
        // std::cout << "mMinra*sin(startangle1 + symbol1*iangle) = " << mMinra*sin(startangle1 + symbol1*iangle) << std::endl;

        // std::cout << "startangle1 + symbol1*iangle= " << startangle1 + symbol1*iangle << std::endl;
        // std::cout << "startangle1= " << startangle1 << std::endl;

        InsertPubPointEA(tempoint);
    }

    if(!havecollition){
        double tempgradient;
        tempgradient = atan2(point2[1] - point1[1], point2[0] - point1[0]);
        for (double ilength = 0; ilength < l2; ilength += 0.2){
            tempoint[0] = point1[0] + ilength*cos(tempgradient);
            tempoint[1] = point1[1] + ilength*sin(tempgradient);
            // std::cout << "centre1[0] = " << centre1[0] << std::endl;
            // std::cout << "centre1[1] = " << centre1[1] << std::endl;
            InsertPubPointEA(tempoint);
        }
    }

    if(!havecollition){
        double startangle3 = theta3; 
        for (double iangel = 0; iangel < alpha2; iangel += 0.02){
            tempoint[0] = centre2[0] + mMinra*cos(startangle3 + symbol2*iangel);
            tempoint[1] = centre2[1] + mMinra*sin(startangle3 + symbol2*iangel);
            
            InsertPubPointEA(tempoint);
        }
    }

    if(havecollition){
        mDubinsPath.poses.clear();
        // std::cout << "撞障碍物了 ！" << std::endl;
        lineListS.insert({inf,type});
        return inf;
    }
    else{
        mtempPathList[type] = mDubinsPath;
        lineListS.insert({S,type});
        // std::cout << "type =  " << type << "好了！！"<<std::endl;
        // std::cout << "type的 SS =  " << S << std::endl;
        return S;
    }

}

// void tydubins::GetPointOfContactPub(const Eigen::Vector2d point1,const Eigen::Vector2d point2, 
// const Eigen::Vector2d point3, const Eigen::Vector2d point4,
// const Eigen::Vector2d point5,const Eigen::Vector2d point6, 
// const Eigen::Vector2d point7, const Eigen::Vector2d point8)
// {
//     geometry_msgs::Point temp1;
//     geometry_msgs::Point temp2;
//     geometry_msgs::Point temp3;
//     geometry_msgs::Point temp4;
//     geometry_msgs::Point temp5;
//     geometry_msgs::Point temp6;
//     geometry_msgs::Point temp7;
//     geometry_msgs::Point temp8;

//     Make_point1.header.frame_id = "map";
//     Make_point1.type = visualization_msgs::Marker::POINTS;
//     Make_point1.scale.x = 10;
//     Make_point1.scale.y = 10;
//     Make_point1.scale.z = 10;
//     Make_point1.color.a = 0.8;
//     Make_point1.color.r = 0;
//     Make_point1.color.g = 0;
//     Make_point1.color.b = 1.0;

//     temp1.x = point1[0]; temp1.y = point1[1];
//     temp2.x = point2[0]; temp2.y = point2[1];
//     temp3.x = point3[0]; temp3.y = point3[1];
//     temp4.x = point4[0]; temp4.y = point4[1];
//     temp5.x = point5[0]; temp5.y = point5[1];
//     temp6.x = point6[0]; temp6.y = point6[1];
//     temp7.x = point7[0]; temp7.y = point7[1];
//     temp8.x = point8[0]; temp8.y = point8[1];

//     Make_point1.points.clear();
//     Make_point1.points.push_back(temp1);
//     Make_point1.points.push_back(temp2);
//     Make_point1.points.push_back(temp3);
//     Make_point1.points.push_back(temp4);
//     Make_point1.points.push_back(temp5);
//     Make_point1.points.push_back(temp6);
//     Make_point1.points.push_back(temp7);
//     Make_point1.points.push_back(temp8);

// }