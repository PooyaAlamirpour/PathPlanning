//
// Created by pooya on 4/4/20.
//

#ifndef PATH_PLANNING_WAYPOINT_H
#define PATH_PLANNING_WAYPOINT_H

#include <vector>
using namespace std;

class WayPoint {
public:
    WayPoint();
    vector<double> coarse_waypoints_s;
    vector<double> coarse_waypoints_x;
    vector<double> coarse_waypoints_y;
    vector<double> coarse_waypoints_dx;
    vector<double> coarse_waypoints_dy;

    void updateCoarse();
};


#endif //PATH_PLANNING_WAYPOINT_H
