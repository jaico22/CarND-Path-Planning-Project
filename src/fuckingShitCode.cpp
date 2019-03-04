#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <math.h>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


int main() {
  uWS::Hub h;
  const int STATE_KEEP_LANE = 0;
  int PathPlanningState = STATE_KEEP_LANE;
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  double ref_s;
  double ref_d;
  double ref_theta; 
  double vehicle_sped = 0.0; 
  string behaviorState = "keep";
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&vehicle_sped,&behaviorState]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

                      // Preventing collitions.
            if (previous_path_x.size() > 0) {
              car_s = end_path_s;
            }
          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          std::cout << "S: " << car_s << " D: " << car_d << std::endl;
          std::cout << "X: " << car_x << " Y: " << car_y << std::endl;

          // Sensor Fusion

          const double max_veh_speed = 45; 

          double target_v = max_veh_speed; 
          double closestCar_vy;
          double closestCar_s = 100000000; 
          double closestCar_vx;
          int currentLane = 1;//round((car_d-2)/4);
          if (behaviorState == "keep"){
            // Match speed of nearest vehicle
            vector<double> closestCar;
            // Find closes car in lane
            std::cout << "Number of cars: " << sensor_fusion.size() << std::endl;
            for (int i = 0; i < sensor_fusion.size(); i++){

              double sf_car_d = sensor_fusion[i][6];
              double sf_car_s = sensor_fusion[i][5];
              int vehicleLane = round((sf_car_d-2)/4);
              std::cout << "Car Lane = " << currentLane << " Other Veh Lane = " << vehicleLane << std::endl;
              std::cout << "  s = " << sf_car_s << " d = " << sf_car_d << std::endl;
              if ((vehicleLane == currentLane)&(sf_car_s>car_s)&(sf_car_s<closestCar_s)){
                closestCar_vx = sensor_fusion[i][3];
                closestCar_vy = sensor_fusion[i][4];
                closestCar_s = sensor_fusion[i][5];
              }
            }
            if (fabs(car_s - closestCar_s) < 100.0){
              double v_yaw = atan2(closestCar_vy,closestCar_vx);
              double delta_yaw = fabs(deg2rad(car_yaw) - v_yaw);
              double v_car = sqrt(pow(closestCar_vy,2)+pow(closestCar_vx,2));
              // Unit's arean't exactl documented... this seems to work okay??
              const double MAGIC_UNIT_CONV = 45/18;
              double closestCarV = v_car *sin(3.14159/2 - delta_yaw) * MAGIC_UNIT_CONV;
              target_v = closestCarV;
              std::cout << "Closest Car --- " << std::endl;
              std::cout << "  vx = " << closestCar_vx << " vy = " << closestCar_vy << std::endl;
              std::cout << "  Car Yaw: " << deg2rad(car_yaw) << " Closest Yaw = " << v_yaw << std::endl;
              std::cout << "  VTangental = " << closestCarV << std::endl;
              
              std::cout << "Closest Car V = " << closestCarV << std::endl;
            }
            
          }
          // Speed control
          double delta_speed = 0.0;
          if (vehicle_sped < target_v) {
            delta_speed += 0.2; 
          }else if(vehicle_sped > target_v){
            delta_speed -= 0.2; 
          }
          /*if (car_speed > max_veh_speed){
            delta_speed -= 0.20; 
          }*/
          std::cout << "Current speed: " << vehicle_sped << std::endl;
          // Lane control
          double targetLane = 1;
          
          // ------------ Tragectory Generation ---------------
          double x0 = car_x;
          double y0 = car_y;
          double theta0 = deg2rad(car_yaw); 
          vector<double> xPntsSpline;
          vector<double> yPntsSpline;
          if (previous_path_y.size() < 2){
              std::cout << "Yaw = " << car_yaw << std::endl;
              xPntsSpline.push_back(x0 - cos((theta0)));
              yPntsSpline.push_back(y0 - sin((theta0)));

              xPntsSpline.push_back(x0);
              yPntsSpline.push_back(y0);

          }else{
            double prevPathY2 = previous_path_y[previous_path_y.size()-2];
            double prevPathX2 = previous_path_x[previous_path_y.size()-2];
            x0 = previous_path_x[previous_path_y.size()-1];
            y0 = previous_path_y[previous_path_y.size()-1];

            xPntsSpline.push_back(previous_path_x[previous_path_y.size()-2]);
            yPntsSpline.push_back(previous_path_y[previous_path_y.size()-2]);  
            std::cout << "    (" << prevPathX2 << "," << prevPathY2 << ")" << std::endl; 
            if (fabs(prevPathX2 - x0) > 1){
              xPntsSpline.push_back(x0);
              yPntsSpline.push_back(y0);
              std::cout << "    (" << x0 << "," << y0 << ")" << std::endl; 
            }
            if (fabs(y0-prevPathY2) > 1){
              theta0 = atan2(y0 - prevPathY2,x0 - prevPathX2) - 3.14159/2;
            }else{
              theta0 = 0.0;
            }
            
            //std::cout << "y0 - 1 = " << prevPathY2 << " y0 = " << y0  << std::endl;
            //std::cout << "x0 - 1 = " << prevPathX2 << " x0 = " << x0 << std::endl;
            
            std::cout << "New Theta: " << theta0 << std::endl;
         
          }
          double spacing = 25;
          std::cout << " -- Way Points -- " << std::endl;
          vector<double> wp0 = getXY(car_s+spacing,2+4*targetLane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          std::cout << "wp0: (" << wp0[0] << "," << wp0[1] << ")" << std::endl;
          xPntsSpline.push_back(wp0[0]);
          yPntsSpline.push_back(wp0[1]);
          vector<double> wp1 = getXY(car_s+2*spacing,2+4*targetLane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          xPntsSpline.push_back(wp1[0]);
          yPntsSpline.push_back(wp1[1]);    
          std::cout << "wp1: (" << wp1[0] << "," << wp1[1] << ")" << std::endl;    
          vector<double> wp2 = getXY(car_s+3*spacing,2+4*targetLane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          xPntsSpline.push_back(wp2[0]);
          yPntsSpline.push_back(wp2[1]);    
          std::cout << "wp2: (" << wp2[0] << "," << wp2[1] << ")" << std::endl;   
          // Shift coordinate system
          for (int i = 0; i < xPntsSpline.size(); i++){
            //std::cout << "X = " << xPntsSpline[i] << " Y = " << yPntsSpline[i] << std::endl;       

            xPntsSpline[i] = (xPntsSpline[i]-x0) * cos(-theta0) - 
                              (yPntsSpline[i]-y0) * sin(-theta0);
            yPntsSpline[i] = (xPntsSpline[i]-x0) * sin(-theta0) + 
                              (yPntsSpline[i]-y0) * cos(-theta0);  
            //std::cout << " --> X = " << xPntsSpline[i] << " Y = " << yPntsSpline[i] << std::endl;       
          }

          tk::spline s;
          //std::cout << "Creating spline.." << std::endl;
          s.set_points(xPntsSpline,yPntsSpline);

          double targetY = s(spacing);
          //std::cout << "Target X: " << spacing << " Target Y: " << targetY << std::endl; 
          double targetDist = sqrt(pow(spacing,2)+pow(targetY,2));
          //std::cout << "Target dist: " << targetDist << std::endl;
          double x_ref = 0;
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          std::cout << "v = " << vehicle_sped << std::endl;
          double y = 0;
          double x = 0;
          double y_ref = 0;
          double allowedMovement = 2 * spacing / (targetDist / (0.02*vehicle_sped/2.24)); 
          for(int i = 1; i < 50 - previous_path_x.size(); i++){
            double x_tmp = 0.0;
            double y_tmp = 0.0;
            double spacingtmp = spacing;
            vehicle_sped += delta_speed;
            if (vehicle_sped > max_veh_speed){
              vehicle_sped = max_veh_speed;
            }else if (vehicle_sped < 0.2){
              vehicle_sped = 0.2;
            }
            double speedMult = (0.02*vehicle_sped/2.24);
            x_tmp = x_ref + spacingtmp / (targetDist / speedMult);
            y_tmp = s(x_tmp); 
            // Modify speed if too fast
            double modSpeed = sqrt(pow(x_tmp - x_ref,2)+pow(y_tmp-y_ref,2));
            double movAngle = atan2(x_tmp-x_ref,y_tmp-y_ref);
            x_tmp = x_tmp - sin(movAngle)*(modSpeed - speedMult);
            y_tmp = y_tmp - cos(movAngle)*(modSpeed - speedMult);
            double modSpeed2 = sqrt(pow(x_tmp - x_ref,2)+pow(y_tmp-y_ref,2));
            std::cout << "Speed0 = " << modSpeed << " --> " << modSpeed2 << std::endl;
            x_ref = x_tmp;
            y_ref = y_tmp;
            x = x_ref * cos(theta0) - y_ref * sin(theta0) + x0;
            y = x_ref * sin(theta0) + y_ref * cos(theta0) + y0;
            //std::cout << "x=" << x << " y=" << y << std::endl;
            next_x_vals.push_back(x);
            next_y_vals.push_back(y);
          }
          //std::cout << " --- Path --- " << std::endl;
          //for(int i = 0; i < next_x_vals.size(); i++){
           // std::cout << "x = " << next_x_vals[i] << " y = " << next_y_vals[i] << std::endl;
          //}
          std::cout << std::endl;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}