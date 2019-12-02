#include <vector>
#include <array>
#include <fstream>
#include <ros/ros.h>

#include "cvxgen/solver.h"
#include "mpc_av/mpc_solver.h"

/// Global Variable for CVXGEN solvers
Vars vars;
Params params;
Workspace work;
Settings settings;

static const double x_min = 1.3;
static const double x_max = 1.6;
static const double y_min =-1.5;
static const double y_max = 1.5;
static const double x_dt = 0.2;
static const double y_dt = 0.2;
static const std::array<double, 5> theta_range = {-0.2, -0.1, 0.0, 0.1, 0.2};

/// Format of CSV Generated
/// x_curr, y_curr,
/// input_steering_angle,
/// x1, y1, x2, y2, ...., x10, y10

int main(int argc, char **argv)
{
    ros::init(argc, argv, "generate_trajectories");
    ros::NodeHandle nh;

    double B;
    nh.getParam("B", B);

    std::array<double, 4> Q{};
    nh.getParam("Q0", Q[0]);
    nh.getParam("Q1", Q[1]);
    nh.getParam("Q2", Q[2]);
    nh.getParam("Q3", Q[3]);

    std::array<double, 4> Qf{};
    nh.getParam("Q0", Qf[0]);
    nh.getParam("Q1", Qf[1]);
    nh.getParam("Q2", Qf[2]);
    nh.getParam("Q3", Qf[3]);

    std::array<double, 4> R{};
    nh.getParam("Q0", R[0]);
    nh.getParam("Q1", R[1]);
    nh.getParam("Q2", R[2]);
    nh.getParam("Q3", R[3]);

    double S;
    nh.getParam("S", S);

    std::ofstream trajectories_file;
    trajectories_file.open("/home/yash/yasht_ws/src/mpc_av/trajectories.csv");

    double x_curr = x_min;
    double y_curr = y_min;

    mpc::MPCSolver solver(Q, Qf, R, B, S);
    settings.verbose = 0;
    settings.eps = 1e-4;

    while(x_curr < x_max)
    {
        while(y_curr < y_max)
        {
            for(const auto& theta: theta_range)
            {
                const std::array<double, 3> current_goal{y_curr, x_curr, theta};
                const double input = solver.solve_mpc(current_goal);
                trajectories_file << x_curr << ", " << y_curr << ", " << theta << ", \n";
                trajectories_file << input << ", \n";
                for(int i=1; i<11; i++)
                {
                    trajectories_file << vars.x[i][1] << ", " << vars.x[i][0] << ", ";
                }
                trajectories_file << "\n";
            }
            y_curr = y_curr + y_dt;
        }
        y_curr = y_min;
        x_curr = x_curr + x_dt;
    }

    trajectories_file.close();
}

