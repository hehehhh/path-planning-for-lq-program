#ifndef _CAR_ODOM_H_
#define _CAR_ODOM_H_

#include <vector>

struct car_model
{
    //car position
    double X;
    double Y;

    //car linear

    double linear_x = 1.0;
    double linear_y;

    //car orientation
    double euler_z;

    // std::vector<double> quatation;

};


#endif
