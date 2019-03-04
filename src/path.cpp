
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include <cmath>
#include <iostream>

#include "path.h"

using std::vector;

Path::Path() {}

void Path::generatePath(double car_x,double car_y,double car_yaw){

    xVals.clear();
    yVals.clear();

    double dist_inc = 0.5;
    for (int i = 0; i < 50; ++i){
        std::cout << "carx" << car_x << std::endl;
        xVals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
        yVals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
    }
}