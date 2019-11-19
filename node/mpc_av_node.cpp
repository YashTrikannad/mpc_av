#include <ros/ros.h>
#include "mpc_av/follow_waypoints.h"
#include "mpc_av/mpc_solver.h"
#include "cvxgen/solver.h"

/// Global Variable for CVXGEN solvers
Vars vars;
Params params;
Workspace work;
Settings settings;

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_av");
    ros::NodeHandle nh;
    FollowWaypoints follower;
    ros::spin();
    return 0;
}


