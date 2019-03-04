#ifndef PATH_H
#define PATH_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

using std::vector;

class Path
{
    public:
        vector<double> xVals;
        vector<double> yVals;
        int state = 0;
        double target_s;
        double target_d; 
        //Constructor
        Path() {};
        // Destructor
        ~Path() {};
        void generatePath(double car_x,double car_y,double car_yaw);
          
    private:
        int numXYVals = 10;
        // State definitions
        const int STATE_INIT = 0;
        const int STATE_KEEP_LANE = 1;
        // Lane Definitions
        const int LANE_0 = 0; 
        
};

#endif // PATH_H