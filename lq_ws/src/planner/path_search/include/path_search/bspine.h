#ifndef BSPINE_H_
#define BSPINE_H_

#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <cmath>
#include <algorithm>
class tybspline
{

private:
    ros::Rate rate = 1;

    // a col represent a point
    Eigen::MatrixXd mcontrol_points;
    Eigen::VectorXd mknot;
    int p_ = 3, n_, m_;
    double interval_; 
    
public:
    tybspline();
    ~tybspline();
    bool init(const Eigen::MatrixXd &points, const int &degree);

    void setKnot(const Eigen::VectorXd &knot);
    Eigen::VectorXd getKnot();
    Eigen::MatrixXd getControlPoints();
    void setUniformKnot();
    Eigen::VectorXd evaluateDeBoor(const double &u); 
    
};

#endif

